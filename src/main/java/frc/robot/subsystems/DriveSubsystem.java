// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Rotation override allows another piece of code (like a vision aim command)
  // to supply rotation while leaving joystick translation in control.
  private boolean m_rotationOverrideActive = false;
  private double m_rotationOverrideValue = 0.0;
  // Track last requested translation magnitude (0..1) so other commands can adapt.
  private double m_lastTranslationMagnitude = 0.0;
  private final Camara m_camara;
  private boolean m_autoAimEnabled = false;
  private final SlewRateLimiter m_autoAimRotLimiter = new SlewRateLimiter(4.0);
  private static final double kAutoAimKp = -0.02;
  private static final double kAutoAimDeadbandDeg = 1.5;
  private static final double kAutoAimMaxRotCmd = 0.35;
  // Create MAXSwerveModules
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

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
    getGyroRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Camara camara) {
    m_camara = camara;
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    // Calibrate gyro on startup; keep the robot still during this time.
    m_gyro.calibrate();
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    updateAutoAimRotation();
    // Update the odometry in the periodic block
    m_odometry.update(
    getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  public synchronized void setAutoAimEnabled(boolean enabled) {
    m_autoAimEnabled = enabled;
    if (!enabled) {
      clearRotationOverride();
      m_autoAimRotLimiter.reset(0.0);
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

    if (!m_camara.hasTag1()) {
      // Mantiene el modo activo para reenganchar cuando vuelva a detectar el tag.
      setRotationOverride(0.0);
      m_autoAimRotLimiter.reset(0.0);
      SmartDashboard.putBoolean("AutoAim/TrackingTag1", false);
      return;
    }

    SmartDashboard.putBoolean("AutoAim/TrackingTag1", true);
    double yawErrorDeg = MathUtil.inputModulus(m_camara.getTag1YawDeg(), -180.0, 180.0);
    double rotCmd = 0.0;
    if (Math.abs(yawErrorDeg) > kAutoAimDeadbandDeg) {
      rotCmd = MathUtil.clamp(yawErrorDeg * kAutoAimKp, -kAutoAimMaxRotCmd, kAutoAimMaxRotCmd);
    }

    double smoothRotCmd = m_autoAimRotLimiter.calculate(rotCmd);
    setRotationOverride(smoothRotCmd);
    SmartDashboard.putNumber("AutoAim/YawErrorDeg", yawErrorDeg);
    SmartDashboard.putNumber("AutoAim/RotCmd", smoothRotCmd);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
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
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
        getGyroRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Request that future drive calls use this rotation value instead of operator rotation.
   * Call {@link #clearRotationOverride()} to stop overriding.
   * @param rot rotation fraction in [-1,1]
   */
  public synchronized void setRotationOverride(double rot) {
    m_rotationOverrideActive = true;
    m_rotationOverrideValue = Math.max(-1.0, Math.min(1.0, rot));
  }

  /**
   * Clear any rotation override so the operator regains rotation control.
   */
  public synchronized void clearRotationOverride() {
    m_rotationOverrideActive = false;
  }

  /**
   * Helper used by default drive code to pick the rotation input: the override if active,
   * otherwise the requested operator rotation.
   * @param requested the operator rotation input (fraction)
   * @return the rotation to actually use
   */
  public synchronized double getRotationForDrive(double requested) {
    return m_rotationOverrideActive ? m_rotationOverrideValue : requested;
  }

  /**
   * Store the most recent translation magnitude (0..1). Call from the default drive
   * command so aim code can reduce rotation when translation is high.
   */
  public synchronized void setLastTranslationMagnitude(double mag) {
    m_lastTranslationMagnitude = Math.max(0.0, Math.min(1.0, mag));
  }

  /**
   * Returns last reported translation magnitude in [0,1].
   */
  public synchronized double getLastTranslationMagnitude() {
    return m_lastTranslationMagnitude;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
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
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(DriveConstants.kGyroYawAxis) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the gyro angle as a Rotation2d, applying inversion and mounting offset.
   */
  private Rotation2d getGyroRotation() {
    double angle = getGyroAngleDegrees();
    double adjusted = (DriveConstants.kGyroReversed ? -angle : angle)
        + DriveConstants.kGyroMountingOffsetDeg;
    return Rotation2d.fromDegrees(adjusted);
  }

  /**
   * Returns the raw IMU angle converted to degrees.
   */
  private double getGyroAngleDegrees() {
    double angle = m_gyro.getAngle(DriveConstants.kGyroYawAxis);
    return DriveConstants.kGyroAngleIsRotations ? Units.rotationsToDegrees(angle) : angle;
  }

  /**
   * Diagnostic: print raw and effective turning encoder positions for each module
   * and the angle relative to the chassis offset (in degrees). Use this to
   * determine any 90° misalignment.
   */
  public void logTurningEncoders() {
    double flRaw = m_frontLeft.getRawTurningEncoderPosition();
    double frRaw = m_frontRight.getRawTurningEncoderPosition();
    double rlRaw = m_rearLeft.getRawTurningEncoderPosition();
    double rrRaw = m_rearRight.getRawTurningEncoderPosition();

    double flEff = m_frontLeft.getEffectiveTurningEncoderPosition();
    double frEff = m_frontRight.getEffectiveTurningEncoderPosition();
    double rlEff = m_rearLeft.getEffectiveTurningEncoderPosition();
    double rrEff = m_rearRight.getEffectiveTurningEncoderPosition();

    double flRel = flEff - DriveConstants.kFrontLeftChassisAngularOffset;
    double frRel = frEff - DriveConstants.kFrontRightChassisAngularOffset;
    double rlRel = rlEff - DriveConstants.kBackLeftChassisAngularOffset;
    double rrRel = rrEff - DriveConstants.kBackRightChassisAngularOffset;

    System.out.println("=== Swerve turning encoder diagnostics ===");
    System.out.printf("FrontLeft (CAN %d)  raw=%.3f rad (%.1f°)  eff=%.3f rad (%.1f°)  rel=%.1f°\n",
        DriveConstants.kFrontLeftTurningCanId, flRaw, Math.toDegrees(flRaw), flEff, Math.toDegrees(flEff),
        Math.toDegrees(flRel));
    System.out.printf("FrontRight (CAN %d) raw=%.3f rad (%.1f°)  eff=%.3f rad (%.1f°)  rel=%.1f°\n",
        DriveConstants.kFrontRightTurningCanId, frRaw, Math.toDegrees(frRaw), frEff, Math.toDegrees(frEff),
        Math.toDegrees(frRel));
    System.out.printf("RearLeft (CAN %d)   raw=%.3f rad (%.1f°)  eff=%.3f rad (%.1f°)  rel=%.1f°\n",
        DriveConstants.kRearLeftTurningCanId, rlRaw, Math.toDegrees(rlRaw), rlEff, Math.toDegrees(rlEff),
        Math.toDegrees(rlRel));
    System.out.printf("RearRight (CAN %d)  raw=%.3f rad (%.1f°)  eff=%.3f rad (%.1f°)  rel=%.1f°\n",
        DriveConstants.kFrontRightTurningCanId, rrRaw, Math.toDegrees(rrRaw), rrEff, Math.toDegrees(rrEff),
        Math.toDegrees(rrRel));
    System.out.println("(rel = effective encoder - chassis offset)");
  }
}
