package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;

import edu.wpi.first.hal.FRCNetComm.tResourceType;

import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.MathUtil;


import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

public class DriveSubsystem extends SubsystemBase {

  private boolean m_rotationOverrideActive = false;

  private double m_rotationOverrideValue = 0.0;

  private double m_lastTranslationMagnitude = 0.0;

  private final Camara m_camara;

  private boolean m_autoAimEnabled = false;

  private static final double kAutoAimKp = 9.0; // rad/s per rad

  private static final double kAutoAimKd = 0.65; // rad/s per (rad/s)

  private static final double kAutoAimDeadbandDeg = 0.8;

  private static final double kAutoAimMaxRotCmd = 1.0;

  private static final double kMaxLeadDistanceMeters = 4.0;

  private static final double kMinLeadDistanceMeters = 0.25;

  private static final double kShotFlightTimeSec = 0.60;

  private static final double kAccelTimeGainSecPerMps2 = 0.04;

  private static final double kMaxEffectiveFlightTimeSec = 1.20;

  private static final double kVisionLatencySec = 0.015;

  private static final double kAutoAimFeedforwardGain = 1.6;

  private static final double kAccelGToMps2 = 9.80665;

  private double m_prevYawErrorRad = 0.0;

  private double m_prevYawTimestampSec = Timer.getFPGATimestamp();

  // Última velocidad del chasis (robot-relative), actualizada en drive().

  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(

      DriveConstants.kFrontLeftDrivingCanId,

      DriveConstants.kFrontLeftTurningCanId,

      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(

      DriveConstants.kFrontRightDrivingCanId,

      DriveConstants.kFrontRightTurningCanId,

      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(

      DriveConstants.kRearLeftDrivingCanId,

      DriveConstants.kRearLeftTurningCanId,

      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(

      DriveConstants.kRearRightDrivingCanId,

      DriveConstants.kRearRightTurningCanId,

      DriveConstants.kBackRightChassisAngularOffset);

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(

      DriveConstants.kDriveKinematics,

      getGyroRotation(),

      new SwerveModulePosition[] {

          m_frontLeft.getPosition(),

          m_frontRight.getPosition(),

          m_rearLeft.getPosition(),

          m_rearRight.getPosition()

      });

  public DriveSubsystem(Camara camara) {

    m_camara = camara;

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    m_gyro.calibrate();

    m_gyro.reset();

  }

  @Override

  public void periodic() {

    updateAutoAimRotation();

    m_odometry.update(

        getGyroRotation(),

        new SwerveModulePosition[] {

            m_frontLeft.getPosition(),

            m_frontRight.getPosition(),

            m_rearLeft.getPosition(),

            m_rearRight.getPosition()

        });

    Pose2d pose = m_odometry.getPoseMeters();

    SmartDashboard.putNumber("Drive/HeadingDeg", getHeading());

    SmartDashboard.putNumber("Drive/TurnRateDps", getTurnRate());

    SmartDashboard.putNumber("Drive/PoseX", pose.getX());

    SmartDashboard.putNumber("Drive/PoseY", pose.getY());

    SmartDashboard.putNumber("Drive/PoseDeg", pose.getRotation().getDegrees());

  }

  public synchronized void setAutoAimEnabled(boolean enabled) {

    m_autoAimEnabled = enabled;

    if (!enabled) {

      clearRotationOverride();

      m_prevYawErrorRad = 0.0;

      m_prevYawTimestampSec = Timer.getFPGATimestamp();

    }

  }

  public synchronized boolean isAutoAimEnabled() {

    return m_autoAimEnabled;

  }

  private void updateAutoAimRotation() {

    if (!m_autoAimEnabled) {

      SmartDashboard.putBoolean("AutoAim/Enabled", false);

      return;

    }

    SmartDashboard.putBoolean("AutoAim/Enabled", true);

    if (!m_camara.hasAutoAimTag()) {

      setRotationOverride(0.0);

      m_prevYawErrorRad = 0.0;

      m_prevYawTimestampSec = Timer.getFPGATimestamp();

      SmartDashboard.putBoolean("AutoAim/TrackingAutoAimTag", false);

      return;

    }

    SmartDashboard.putBoolean("AutoAim/TrackingAutoAimTag", true);

    SmartDashboard.putNumber("AutoAim/TagId", m_camara.getAutoAimTagId());

  double yawErrorDegRaw = -MathUtil.inputModulus(m_camara.getAutoAimYawDeg(), -180.0, 180.0);

    double gyroRateDegPerSec = getTurnRate();

    double yawErrorDeg = MathUtil.inputModulus(

        yawErrorDegRaw - (gyroRateDegPerSec * kVisionLatencySec),

        -180.0,

        180.0);

    double dist = m_camara.getAutoAimDistanceM();

    double accelLatMps2 = m_gyro.getAccelY() * kAccelGToMps2;

    double tEffSec = kShotFlightTimeSec + (kAccelTimeGainSecPerMps2 * Math.abs(accelLatMps2));

    tEffSec = MathUtil.clamp(tEffSec, 0.0, kMaxEffectiveFlightTimeSec);

    ChassisSpeeds speeds = getLastChassisSpeeds();

    double vLat = speeds.vyMetersPerSecond;

    double omegaFF = 0.0;

  double leadRad = 0.0;

    boolean leadActive = false;

    if (dist > kMinLeadDistanceMeters && dist <= kMaxLeadDistanceMeters) {

      omegaFF = (vLat / dist) * kAutoAimFeedforwardGain;

      leadActive = true;

    }

    double yawErrorRad = Math.toRadians(yawErrorDeg);

    double now = Timer.getFPGATimestamp();

    double dt = Math.max(1e-3, now - m_prevYawTimestampSec);

    double yawErrorDotRad = (yawErrorRad - m_prevYawErrorRad) / dt;

    m_prevYawErrorRad = yawErrorRad;

    m_prevYawTimestampSec = now;

    double omegaCmdRadPerSec = (kAutoAimKp * yawErrorRad) + (kAutoAimKd * yawErrorDotRad) + omegaFF;

    double rotCmd = 0.0;

    if (Math.abs(yawErrorDeg) > kAutoAimDeadbandDeg) {

      rotCmd = MathUtil.clamp(omegaCmdRadPerSec / DriveConstants.kMaxAngularSpeed,

          -kAutoAimMaxRotCmd,

          kAutoAimMaxRotCmd);

    }

    setRotationOverride(rotCmd);

    SmartDashboard.putNumber("AutoAim/YawErrorDegRaw", yawErrorDegRaw);

    SmartDashboard.putNumber("AutoAim/YawErrorDeg", yawErrorDeg);

    SmartDashboard.putNumber("AutoAim/GyroRateDps", gyroRateDegPerSec);

    SmartDashboard.putBoolean("AutoAim/LeadActive", leadActive);

    SmartDashboard.putNumber("AutoAim/DistanceM", dist);

    SmartDashboard.putNumber("AutoAim/VLatMps", vLat);

    SmartDashboard.putNumber("AutoAim/AccelLatMps2", accelLatMps2);

    SmartDashboard.putNumber("AutoAim/LeadTEffSec", tEffSec);

  SmartDashboard.putNumber("AutoAim/LeadRad", leadRad);

    SmartDashboard.putNumber("AutoAim/OmegaFFRadPerSec", omegaFF);

    SmartDashboard.putNumber("AutoAim/YawErrorDotRad", yawErrorDotRad);

    SmartDashboard.putNumber("AutoAim/OmegaCmdRadPerSec", omegaCmdRadPerSec);

    SmartDashboard.putNumber("AutoAim/RotCmd", rotCmd);

  }

  /**
   * 
   * Returns the currently-estimated pose of the robot.
   *
   * 
   * 
   * @return The pose.
   * 
   */

  public Pose2d getPose() {

    return m_odometry.getPoseMeters();

  }

  /**
   * 
   * Resets the odometry to the specified pose.
   *
   * 
   * 
   * @param pose The pose to which to set the odometry.
   * 
   */

  public void resetOdometry(Pose2d pose) {

    m_odometry.resetPosition(

        getGyroRotation(),

        new SwerveModulePosition[] {

            m_frontLeft.getPosition(),

            m_frontRight.getPosition(),

            m_rearLeft.getPosition(),

            m_rearRight.getPosition()

        },

        pose);

  }

  /**
   * 
   * Method to drive the robot using joystick info.
   *
   * 
   * 
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * 
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * 
   * @param rot           Angular rate of the robot.
   * 
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   * 
   *                      field.
   * 
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds speeds = fieldRelative

        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())

        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // Guardar robot-relative para compensaciones (proyectiles, etc.)

    m_lastChassisSpeeds = speeds;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(

        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    boolean holdSteering = m_autoAimEnabled;

    m_frontLeft.setDesiredState(swerveModuleStates[0], holdSteering);

    m_frontRight.setDesiredState(swerveModuleStates[1], holdSteering);

    m_rearLeft.setDesiredState(swerveModuleStates[2], holdSteering);

    m_rearRight.setDesiredState(swerveModuleStates[3], holdSteering);

  }

  /**
   * 
   * Crea un comando que mueve al robot una distancia traslacional en coordenadas
   * del CAMPO
   * 
   * (field-relative), sin girar (rot=0). Usa odometria (Pose2d) para parar cuando
   * llega.
   *
   * 
   * 
   * @param dxMeters +X del campo (meters). Campo-X normalmente es "hacia
   *                 adelante" de la alianza azul.
   * 
   * @param dyMeters +Y del campo (meters).
   * 
   * @param speedMps magnitud de velocidad (m/s). Se limita por
   *                 kMaxSpeedMetersPerSecond.
   * 
   */

  public Command driveFieldRelativeDistance(double dxMeters, double dyMeters, double speedMps) {

    return driveFieldRelativeDistanceWithHeading(dxMeters, dyMeters, speedMps, getHeading());

  }

  /**
   * 
   * Igual que driveFieldRelativeDistance, pero mientras se mueve gira
   * automáticamente a un heading objetivo
   * 
   * (en grados, coordenadas de campo). Es ideal para autónomo.
   *
   * 
   * 
   * targetHeadingDeg: 0 = "norte" del campo (si tu gyro está reseteado apuntando
   * al norte), 90 = este, etc.
   * 
   */

  public Command driveFieldRelativeDistanceWithHeading(

      double dxMeters,

      double dyMeters,

      double speedMps,

      double targetHeadingDeg) {

    final double requestedSpeed = Math.abs(speedMps);

    final double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

    final double userMaxV = MathUtil.clamp(requestedSpeed, 0.0, maxSpeed);

    // Tuning (puedes ajustarlo en la cancha)

    final String kMaxVKey = "Drive/MoveCmd/MaxV";

    final String kMaxAKey = "Drive/MoveCmd/MaxA";

    final String kTolKey = "Drive/MoveCmd/ToleranceM";

    final double kDefaultMaxA = 2.0; // m/s^2

    final double kDefaultTol = 0.05; // 5 cm

    // Heading control tuning

    final String kHeadingKpKey = "Drive/MoveCmd/HeadingKp";

    final String kHeadingTolDegKey = "Drive/MoveCmd/HeadingTolDeg";

    final String kHeadingMaxRotKey = "Drive/MoveCmd/HeadingMaxRot";

    final double kDefaultHeadingKp = 0.02; // rotCmd per deg (antes de clamp)

    final double kDefaultHeadingTolDeg = 2.0;

    final double kDefaultHeadingMaxRot = 0.5; // [-1..1]

    return Commands.defer(

        () -> {

          final Pose2d startPose = getPose();

          final Pose2d targetPose = new Pose2d(

              startPose.getX() + dxMeters,

              startPose.getY() + dyMeters,

              startPose.getRotation());

          // Seed dashboard values

          SmartDashboard.putNumber(kMaxVKey, SmartDashboard.getNumber(kMaxVKey, userMaxV));

          SmartDashboard.putNumber(kMaxAKey, SmartDashboard.getNumber(kMaxAKey, kDefaultMaxA));

          SmartDashboard.putNumber(kTolKey, SmartDashboard.getNumber(kTolKey, kDefaultTol));

          SmartDashboard.putNumber(kHeadingKpKey, SmartDashboard.getNumber(kHeadingKpKey, kDefaultHeadingKp));

          SmartDashboard.putNumber(kHeadingTolDegKey,

              SmartDashboard.getNumber(kHeadingTolDegKey, kDefaultHeadingTolDeg));

          SmartDashboard.putNumber(kHeadingMaxRotKey,

              SmartDashboard.getNumber(kHeadingMaxRotKey, kDefaultHeadingMaxRot));

          SmartDashboard.putNumber("Drive/MoveCmd/TargetHeadingDeg", targetHeadingDeg);

          return Commands

              .runEnd(

                  () -> {

                    // Perfil trapezoidal simple en 1D (sobre distancia):

                    // vCmd = min(vMax, sqrt(2*aMax*distRemaining))

                    // Eso hace que conforme se acerca, la velocidad baje suavemente.

                    double vMax = MathUtil.clamp(SmartDashboard.getNumber(kMaxVKey, userMaxV), 0.0, maxSpeed);

                    double aMax = Math.max(0.1, SmartDashboard.getNumber(kMaxAKey, kDefaultMaxA));

                    double tol = Math.max(0.01, SmartDashboard.getNumber(kTolKey, kDefaultTol));

                    double headingKp = SmartDashboard.getNumber(kHeadingKpKey, kDefaultHeadingKp);

                    double headingTolDeg = Math.max(0.25,

                        SmartDashboard.getNumber(kHeadingTolDegKey, kDefaultHeadingTolDeg));

                    double headingMaxRot = MathUtil.clamp(

                        SmartDashboard.getNumber(kHeadingMaxRotKey, kDefaultHeadingMaxRot),

                        0.05,

                        1.0);

                    Pose2d cur = getPose();

                    double errX = targetPose.getX() - cur.getX();

                    double errY = targetPose.getY() - cur.getY();

                    double distErr = Math.hypot(errX, errY);

                    double vCmd = 0.0;

                    if (distErr > tol) {

                      vCmd = Math.min(vMax, Math.sqrt(2.0 * aMax * distErr));

                    }

                    // Dirección hacia el target (field-relative)

                    double vxField = 0.0;

                    double vyField = 0.0;

                    if (distErr > 1e-6) {

                      vxField = (errX / distErr) * vCmd;

                      vyField = (errY / distErr) * vCmd;

                    }

                    // Convertimos a inputs -1..1 para reusar drive().

                    double xCmd = vxField / maxSpeed;

                    double yCmd = vyField / maxSpeed;

                    // Heading: error en grados, con wrap [-180,180]

                    double headingErrDeg = MathUtil.inputModulus(targetHeadingDeg - getHeading(), -180.0, 180.0);

                    double rotCmd = 0.0;

                    if (Math.abs(headingErrDeg) > headingTolDeg) {

                      rotCmd = MathUtil.clamp(headingErrDeg * headingKp, -headingMaxRot, headingMaxRot);

                    }

                    SmartDashboard.putNumber("Drive/MoveCmd/StartX", startPose.getX());

                    SmartDashboard.putNumber("Drive/MoveCmd/StartY", startPose.getY());

                    SmartDashboard.putNumber("Drive/MoveCmd/TargetX", targetPose.getX());

                    SmartDashboard.putNumber("Drive/MoveCmd/TargetY", targetPose.getY());

                    SmartDashboard.putNumber("Drive/MoveCmd/ErrX", errX);

                    SmartDashboard.putNumber("Drive/MoveCmd/ErrY", errY);

                    SmartDashboard.putNumber("Drive/MoveCmd/DistErr", distErr);

                    SmartDashboard.putNumber("Drive/MoveCmd/VMax", vMax);

                    SmartDashboard.putNumber("Drive/MoveCmd/ACmd", aMax);

                    SmartDashboard.putNumber("Drive/MoveCmd/VCmd", vCmd);

                    SmartDashboard.putNumber("Drive/MoveCmd/Tol", tol);

                    SmartDashboard.putNumber("Drive/MoveCmd/HeadingErrDeg", headingErrDeg);

                    SmartDashboard.putNumber("Drive/MoveCmd/RotCmd", rotCmd);

                    // fieldRelative = true

                    drive(xCmd, yCmd, rotCmd, true);

                  },

                  () -> drive(0.0, 0.0, 0.0, true),

                  this)

              .until(() -> {

                double tol = Math.max(0.01, SmartDashboard.getNumber(kTolKey, kDefaultTol));

                Pose2d cur = getPose();

                double errX = targetPose.getX() - cur.getX();

                double errY = targetPose.getY() - cur.getY();

                return Math.hypot(errX, errY) <= tol;

              });

        },

        Set.of(this));

  }

  /**
   * 
   * Velocidad estimada del chasis (robot-relative) de la última orden de manejo.
   * 
   * vx: +adelante (m/s), vy: +izquierda (m/s), omega: rad/s.
   * 
   */

  public synchronized ChassisSpeeds getLastChassisSpeeds() {

    return m_lastChassisSpeeds;

  }

  public synchronized void setRotationOverride(double rot) {

    m_rotationOverrideActive = true;

    m_rotationOverrideValue = Math.max(-1.0, Math.min(1.0, rot));

  }

  public synchronized void clearRotationOverride() {

    m_rotationOverrideActive = false;

  }

  public synchronized double getRotationForDrive(double requested) {

    return m_rotationOverrideActive ? m_rotationOverrideValue : requested;

  }

  public synchronized void setLastTranslationMagnitude(double mag) {

    m_lastTranslationMagnitude = Math.max(0.0, Math.min(1.0, mag));

  }

  public synchronized double getLastTranslationMagnitude() {

    return m_lastTranslationMagnitude;

  }

  public void setX() {

    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));

    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));

    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(

        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);

    m_frontRight.setDesiredState(desiredStates[1]);

    m_rearLeft.setDesiredState(desiredStates[2]);

    m_rearRight.setDesiredState(desiredStates[3]);

  }

  /** Resets the drive encoders to currently read a position of 0. */

  public void resetEncoders() {

    m_frontLeft.resetEncoders();

    m_rearLeft.resetEncoders();

    m_frontRight.resetEncoders();

    m_rearRight.resetEncoders();

  }

  /** Zeroes the heading of the robot. */

  public void zeroHeading() {

    m_gyro.reset();

  }

  /**
   * 
   * Returns the heading of the robot.
   *
   * 
   * 
   * @return the robot's heading in degrees, from -180 to 180
   * 
   */

  public double getHeading() {

    return getGyroRotation().getDegrees();

  }

  /**
   * 
   * Returns the turn rate of the robot.
   *
   * 
   * 
   * @return The turn rate of the robot, in degrees per second
   * 
   */

  public double getTurnRate() {

    return m_gyro.getRate(DriveConstants.kGyroYawAxis) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

  }

  /**
   * 
   * Returns the gyro angle as a Rotation2d, applying inversion and mounting
   * offset.
   * 
   */

  private Rotation2d getGyroRotation() {

    double angle = getGyroAngleDegrees();

    double adjusted = (DriveConstants.kGyroReversed ? -angle : angle)

        + DriveConstants.kGyroMountingOffsetDeg;

    return Rotation2d.fromDegrees(adjusted);

  }

  /**
   * 
   * Returns the raw IMU angle converted to degrees.
   * 
   */

  private double getGyroAngleDegrees() {

    double angle = m_gyro.getAngle(DriveConstants.kGyroYawAxis);

    return DriveConstants.kGyroAngleIsRotations ? Units.rotationsToDegrees(angle) : angle;

  }

}