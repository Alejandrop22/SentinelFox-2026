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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
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
    private static final double kAutoAimKp = 7.5;
    private static final double kAutoAimKd = 0.1;
    private static final double kAutoAimDeadbandDeg = 2.5;
    private static final double kAutoAimMaxRotCmd = 0.50;
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

    // Estado para estimar aceleración lateral (dv/dt) y mejorar el lead.
    private double m_prevVLatMps = 0.0;
    private double m_prevVLatTimestampSec = Timer.getFPGATimestamp();

    // Filtros para quitar el "steppy" del lead (especialmente en vy y dv/dt).
    private double m_vLatFiltMps = 0.0;
    private double m_aLatFiltMps2 = 0.0;

    // Suaviza el comando de rotación en AutoAim para evitar que "se trabe" por saltos.
    // Un valor de 6 (1/s) significa que puede cambiar de -1 a 1 en ~0.33s.
    private final SlewRateLimiter m_autoAimRotLimiter = new SlewRateLimiter(6.0);

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
            m_autoAimRotLimiter.reset(0.0);
            m_prevVLatMps = 0.0;
            m_prevVLatTimestampSec = Timer.getFPGATimestamp();
            m_vLatFiltMps = 0.0;
            m_aLatFiltMps2 = 0.0;
        }
    }

    public synchronized boolean isAutoAimEnabled() {
        return m_autoAimEnabled;
    }

    private void updateAutoAimRotation() {
        // Publicar defaults para que las keys existan en Shuffleboard aunque AutoAim esté apagado
        // o no exista tag (NetworkTables solo muestra entries que alguna vez fueron publicadas).
        SmartDashboard.putBoolean("AutoAim/TrackingAutoAimTag", false);
        SmartDashboard.putNumber("AutoAim/TagId", -1);
        SmartDashboard.putNumber("AutoAim/YawErrorDegRaw", 0.0);
        SmartDashboard.putNumber("AutoAim/YawErrorDeg", 0.0);
        SmartDashboard.putNumber("AutoAim/YawErrorRad", 0.0);
        SmartDashboard.putNumber("AutoAim/YawErrorRadAdjusted", 0.0);
        SmartDashboard.putNumber("AutoAim/YawErrorDegAdjusted", 0.0);
        SmartDashboard.putNumber("AutoAim/GyroRateDps", 0.0);
        SmartDashboard.putBoolean("AutoAim/LeadActive", false);
        SmartDashboard.putNumber("AutoAim/DistanceM", 0.0);
        SmartDashboard.putNumber("AutoAim/VLatMps", 0.0);
        SmartDashboard.putNumber("AutoAim/AccelLatMps2", 0.0);
        SmartDashboard.putNumber("AutoAim/LeadTEffSec", 0.0);
        SmartDashboard.putNumber("AutoAim/LeadMeters", 0.0);
        SmartDashboard.putNumber("AutoAim/LeadRad", 0.0);
        SmartDashboard.putNumber("AutoAim/OmegaFFRadPerSec", 0.0);
        SmartDashboard.putNumber("AutoAim/YawErrorDotRad", 0.0);
        SmartDashboard.putNumber("AutoAim/OmegaCmdRadPerSec", 0.0);
        SmartDashboard.putNumber("AutoAim/RotCmd", 0.0);
        SmartDashboard.putNumber("AutoAim/RotCmdSmoothed", 0.0);

        // Publicar defaults de tuning para que aparezcan siempre.
        SmartDashboard.putNumber("AutoAim/Tuning/Kp", kAutoAimKp);
        SmartDashboard.putNumber("AutoAim/Tuning/Kd", kAutoAimKd);
        SmartDashboard.putNumber("AutoAim/Tuning/DeadbandDeg", kAutoAimDeadbandDeg);
        SmartDashboard.putNumber("AutoAim/Tuning/MaxRotCmd", kAutoAimMaxRotCmd);

        if (!m_autoAimEnabled) {
            SmartDashboard.putBoolean("AutoAim/Enabled", false);
            return;
        }

        SmartDashboard.putBoolean("AutoAim/Enabled", true);
        if (!m_camara.hasAutoAimTag()) {
            setRotationOverride(0.0);
            m_prevYawErrorRad = 0.0;
            m_prevYawTimestampSec = Timer.getFPGATimestamp();
            // Resetear estado de aceleración lateral para evitar un pico al recuperar el tag.
            m_prevVLatMps = 0.0;
            m_prevVLatTimestampSec = Timer.getFPGATimestamp();
            m_vLatFiltMps = 0.0;
            m_aLatFiltMps2 = 0.0;
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
        double now = Timer.getFPGATimestamp();

        // Estimar aceleración lateral (m/s^2) desde el cambio de vy.
        // Se clampa para evitar que el ruido del drivetrain se convierta en lead enorme.
        double dtV = Math.max(1e-3, now - m_prevVLatTimestampSec);
        double aLatRaw = (vLat - m_prevVLatMps) / dtV;

        final double kMaxALatMps2 = 6.0;
        double aLat = MathUtil.clamp(aLatRaw, -kMaxALatMps2, kMaxALatMps2);

        // Filtro IIR simple (0..1): más alto = más rápido pero puede verse "steppy".
        double leadFilterAlpha = SmartDashboard.getNumber("AutoAim/Lead/FilterAlpha", 0.35);
        leadFilterAlpha = MathUtil.clamp(leadFilterAlpha, 0.05, 1.0);
        m_vLatFiltMps += leadFilterAlpha * (vLat - m_vLatFiltMps);
        m_aLatFiltMps2 += leadFilterAlpha * (aLat - m_aLatFiltMps2);

        m_prevVLatMps = vLat;
        m_prevVLatTimestampSec = now;
        double omegaFF = 0.0;
        double leadRad = 0.0;
        double leadMeters = 0.0;
        boolean leadActive = false;

    // Lead settings (tuneables pero seguros por default)
    final double kLeadMinVLatMps = SmartDashboard.getNumber("AutoAim/Lead/MinVLatMps", 0.10);
    final double kLeadMaxDeg = SmartDashboard.getNumber("AutoAim/Lead/MaxDeg", 18.0);
    final double kLeadMaxRad = Math.toRadians(kLeadMaxDeg);

    // Exageración: multiplica cuánto compensamos el lead (esto es lo que pedís).
    // Si a máxima strafe seguís "cayendo" ~1m, subí esto (p.ej. 1.5 -> 2.5).
    double leadGain = SmartDashboard.getNumber("AutoAim/Lead/Gain", 2.0);
    leadGain = MathUtil.clamp(leadGain, 0.0, 5.0);

        if (dist > kMinLeadDistanceMeters && dist <= kMaxLeadDistanceMeters && Math.abs(m_vLatFiltMps) > kLeadMinVLatMps) {
            // Lead dinámico (velocidad + aceleración):
            //   d = v*t + 0.5*a*t^2
            // Esto mejora la sensación de "en tiempo real" cuando el strafe cambia.
            leadMeters = ((m_vLatFiltMps * tEffSec) + (0.5 * m_aLatFiltMps2 * tEffSec * tEffSec)) * leadGain;

            // Sumamos el offset al error (objetivo fantasma). Si vLat>0, el objetivo
            // fantasma queda "adelantado" al lado contrario para compensar.
            leadRad = Math.atan2(-leadMeters, Math.max(dist, 1e-6));
            leadRad = MathUtil.clamp(leadRad, -kLeadMaxRad, kLeadMaxRad);

            // Mantener feedforward existente (opcional), pero ahora puede coexistir con leadRad.
            omegaFF = (m_vLatFiltMps / dist) * kAutoAimFeedforwardGain;
            leadActive = true;
        }

        // Permite tuning en vivo desde Dashboard (si no existen, usa defaults constantes)
        double kp = SmartDashboard.getNumber("AutoAim/Tuning/Kp", kAutoAimKp);
        double kd = SmartDashboard.getNumber("AutoAim/Tuning/Kd", kAutoAimKd);
        double deadbandDeg = SmartDashboard.getNumber("AutoAim/Tuning/DeadbandDeg", kAutoAimDeadbandDeg);
        double maxRotCmd = SmartDashboard.getNumber("AutoAim/Tuning/MaxRotCmd", kAutoAimMaxRotCmd);

        double yawErrorRad = Math.toRadians(yawErrorDeg);
        double yawErrorRadAdjusted = yawErrorRad + leadRad;
        double dt = Math.max(1e-3, now - m_prevYawTimestampSec);
        double yawErrorDotRad = (yawErrorRadAdjusted - m_prevYawErrorRad) / dt;

        m_prevYawErrorRad = yawErrorRadAdjusted;
        m_prevYawTimestampSec = now;

        // Damping con gyro (mucho menos ruidoso que derivar la visión):
        // gyroRateDegPerSec es la velocidad real de giro; en rad/s.
        double gyroRateRadPerSec = Math.toRadians(gyroRateDegPerSec);
        double omegaCmdRadPerSec = (kp * yawErrorRadAdjusted) - (kd * gyroRateRadPerSec) + omegaFF;
        double rotCmd = 0.0;

        if (Math.abs(yawErrorDeg) > deadbandDeg) {
            rotCmd = MathUtil.clamp(omegaCmdRadPerSec / DriveConstants.kMaxAngularSpeed,
                -maxRotCmd,
                maxRotCmd);
        } else {
            // Si estamos dentro del deadband, "congela" el estado del derivativo para que no
            // se acumule y luego pegue un giro fuerte cuando vuelva a salir.
            m_prevYawErrorRad = yawErrorRadAdjusted;
            m_prevYawTimestampSec = now;
        }

        // Suavizar (anti-jerk) para que al entrar/salir del deadband no pegue saltos.
        double rotCmdSmoothed = m_autoAimRotLimiter.calculate(rotCmd);
        setRotationOverride(rotCmdSmoothed);
        SmartDashboard.putNumber("AutoAim/YawErrorDegRaw", yawErrorDegRaw);
        SmartDashboard.putNumber("AutoAim/YawErrorDeg", yawErrorDeg);
        SmartDashboard.putNumber("AutoAim/YawErrorRad", yawErrorRad);
        SmartDashboard.putNumber("AutoAim/YawErrorRadAdjusted", yawErrorRadAdjusted);
        SmartDashboard.putNumber("AutoAim/YawErrorDegAdjusted", Math.toDegrees(yawErrorRadAdjusted));
        SmartDashboard.putNumber("AutoAim/GyroRateDps", gyroRateDegPerSec);
        SmartDashboard.putBoolean("AutoAim/LeadActive", leadActive);
        SmartDashboard.putNumber("AutoAim/DistanceM", dist);
        SmartDashboard.putNumber("AutoAim/VLatMps", vLat);
        SmartDashboard.putNumber("AutoAim/AccelLatMps2", accelLatMps2);
    SmartDashboard.putNumber("AutoAim/VLatFiltMps", m_vLatFiltMps);
    SmartDashboard.putNumber("AutoAim/ALatMps2", m_aLatFiltMps2);
    SmartDashboard.putNumber("AutoAim/ALatRawMps2", aLatRaw);
    SmartDashboard.putNumber("AutoAim/LeadMetersVel", m_vLatFiltMps * tEffSec);
    SmartDashboard.putNumber("AutoAim/LeadMetersAccel", 0.5 * m_aLatFiltMps2 * tEffSec * tEffSec);
        SmartDashboard.putNumber("AutoAim/LeadMetersTotal", leadMeters);
        SmartDashboard.putNumber("AutoAim/LeadTEffSec", tEffSec);
        SmartDashboard.putNumber("AutoAim/LeadMeters", leadMeters);
        SmartDashboard.putNumber("AutoAim/LeadRad", leadRad);
        SmartDashboard.putNumber("AutoAim/LeadMaxDeg", kLeadMaxDeg);
        SmartDashboard.putNumber("AutoAim/LeadMinVLatMps", kLeadMinVLatMps);
        SmartDashboard.putNumber("AutoAim/LeadMaxALatMps2", kMaxALatMps2);
    SmartDashboard.putNumber("AutoAim/LeadGain", leadGain);
    SmartDashboard.putNumber("AutoAim/LeadFilterAlpha", leadFilterAlpha);
        SmartDashboard.putNumber("AutoAim/OmegaFFRadPerSec", omegaFF);
        SmartDashboard.putNumber("AutoAim/YawErrorDotRad", yawErrorDotRad);
        SmartDashboard.putNumber("AutoAim/OmegaCmdRadPerSec", omegaCmdRadPerSec);
        SmartDashboard.putNumber("AutoAim/RotCmd", rotCmd);
        SmartDashboard.putNumber("AutoAim/RotCmdSmoothed", rotCmdSmoothed);

        // Volver a publicar los tuning actuales (con lo que se está usando)
        SmartDashboard.putNumber("AutoAim/Tuning/Kp", kp);
        SmartDashboard.putNumber("AutoAim/Tuning/Kd", kd);
        SmartDashboard.putNumber("AutoAim/Tuning/DeadbandDeg", deadbandDeg);
        SmartDashboard.putNumber("AutoAim/Tuning/MaxRotCmd", maxRotCmd);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

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

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        m_lastChassisSpeeds = speeds;
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        boolean holdSteering = m_autoAimEnabled;
        m_frontLeft.setDesiredState(swerveModuleStates[0], holdSteering);
        m_frontRight.setDesiredState(swerveModuleStates[1], holdSteering);
        m_rearLeft.setDesiredState(swerveModuleStates[2], holdSteering);
        m_rearRight.setDesiredState(swerveModuleStates[3], holdSteering);
    }

    public Command driveFieldRelativeDistance(double dxMeters, double dyMeters, double speedMps) {
        return driveFieldRelativeDistanceWithHeading(dxMeters, dyMeters, speedMps, getHeading());
    }

    public Command driveFieldRelativeDistanceWithHeading(double dxMeters, double dyMeters, double speedMps, double targetHeadingDeg) {
        final double requestedSpeed = Math.abs(speedMps);
        final double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
        final double userMaxV = MathUtil.clamp(requestedSpeed, 0.0, maxSpeed);

        final String kMaxVKey = "Drive/MoveCmd/MaxV";
        final String kMaxAKey = "Drive/MoveCmd/MaxA";
        final String kTolKey = "Drive/MoveCmd/ToleranceM";
        final double kDefaultMaxA = 2.0;
        final double kDefaultTol = 0.05;

        final String kHeadingKpKey = "Drive/MoveCmd/HeadingKp";
        final String kHeadingTolDegKey = "Drive/MoveCmd/HeadingTolDeg";
        final String kHeadingMaxRotKey = "Drive/MoveCmd/HeadingMaxRot";
        final double kDefaultHeadingKp = 0.02;
        final double kDefaultHeadingTolDeg = 2.0;
        final double kDefaultHeadingMaxRot = 0.5;

        return Commands.defer(() -> {
            final Pose2d startPose = getPose();
            final Pose2d targetPose = new Pose2d(
                startPose.getX() + dxMeters,
                startPose.getY() + dyMeters,
                startPose.getRotation());

            SmartDashboard.putNumber(kMaxVKey, SmartDashboard.getNumber(kMaxVKey, userMaxV));
            SmartDashboard.putNumber(kMaxAKey, SmartDashboard.getNumber(kMaxAKey, kDefaultMaxA));
            SmartDashboard.putNumber(kTolKey, SmartDashboard.getNumber(kTolKey, kDefaultTol));
            SmartDashboard.putNumber(kHeadingKpKey, SmartDashboard.getNumber(kHeadingKpKey, kDefaultHeadingKp));
            SmartDashboard.putNumber(kHeadingTolDegKey, SmartDashboard.getNumber(kHeadingTolDegKey, kDefaultHeadingTolDeg));
            SmartDashboard.putNumber(kHeadingMaxRotKey, SmartDashboard.getNumber(kHeadingMaxRotKey, kDefaultHeadingMaxRot));
            SmartDashboard.putNumber("Drive/MoveCmd/TargetHeadingDeg", targetHeadingDeg);

            return Commands.runEnd(() -> {
                double vMax = MathUtil.clamp(SmartDashboard.getNumber(kMaxVKey, userMaxV), 0.0, maxSpeed);
                double aMax = Math.max(0.1, SmartDashboard.getNumber(kMaxAKey, kDefaultMaxA));
                double tol = Math.max(0.01, SmartDashboard.getNumber(kTolKey, kDefaultTol));
                double headingKp = SmartDashboard.getNumber(kHeadingKpKey, kDefaultHeadingKp);
                double headingTolDeg = Math.max(0.25, SmartDashboard.getNumber(kHeadingTolDegKey, kDefaultHeadingTolDeg));
                double headingMaxRot = MathUtil.clamp(SmartDashboard.getNumber(kHeadingMaxRotKey, kDefaultHeadingMaxRot), 0.05, 1.0);

                Pose2d cur = getPose();
                double errX = targetPose.getX() - cur.getX();
                double errY = targetPose.getY() - cur.getY();
                double distErr = Math.hypot(errX, errY);

                double vCmd = (distErr > tol) ? Math.min(vMax, Math.sqrt(2.0 * aMax * distErr)) : 0.0;
                double vxField = (distErr > 1e-6) ? (errX / distErr) * vCmd : 0.0;
                double vyField = (distErr > 1e-6) ? (errY / distErr) * vCmd : 0.0;

                double xCmd = vxField / maxSpeed;
                double yCmd = vyField / maxSpeed;

                double headingErrDeg = MathUtil.inputModulus(targetHeadingDeg - getHeading(), -180.0, 180.0);
                double rotCmd = (Math.abs(headingErrDeg) > headingTolDeg) 
                    ? MathUtil.clamp(headingErrDeg * headingKp, -headingMaxRot, headingMaxRot) : 0.0;

                drive(xCmd, yCmd, rotCmd, true);
            }, () -> drive(0.0, 0.0, 0.0, true), this)
            .until(() -> {
                double tol = Math.max(0.01, SmartDashboard.getNumber(kTolKey, kDefaultTol));
                Pose2d cur = getPose();
                return Math.hypot(targetPose.getX() - cur.getX(), targetPose.getY() - cur.getY()) <= tol;
            });
        }, Set.of(this));
    }

    public synchronized ChassisSpeeds getLastChassisSpeeds() {
        return m_lastChassisSpeeds;
    }

    /**
     * Velocidades robot-relative actuales (lo que se mandó al drivetrain en el último ciclo).
     * PathPlanner usa esto como feedback para el follower.
     */
    public synchronized ChassisSpeeds getRobotRelativeSpeeds() {
        return m_lastChassisSpeeds;
    }

    /**
     * Drive robot-relative (vx, vy, omega). Usado por PathPlanner AutoBuilder.
     * NOTA: no aplica el límite de driver; asume velocidades en m/s y rad/s ya deseadas.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        m_lastChassisSpeeds = speeds;
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    public synchronized void setRotationOverride(double rot) {
        m_rotationOverrideActive = true;
        m_rotationOverrideValue = MathUtil.clamp(rot, -1.0, 1.0);
    }

    public synchronized void clearRotationOverride() {
        m_rotationOverrideActive = false;
    }

    public synchronized double getRotationForDrive(double requested) {
        return m_rotationOverrideActive ? m_rotationOverrideValue : requested;
    }

    public synchronized void setLastTranslationMagnitude(double mag) {
        m_lastTranslationMagnitude = MathUtil.clamp(mag, 0.0, 1.0);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return getGyroRotation().getDegrees();
    }

    public double getTurnRate() {
        return m_gyro.getRate(DriveConstants.kGyroYawAxis) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    private Rotation2d getGyroRotation() {
        double angle = getGyroAngleDegrees();
        double adjusted = (DriveConstants.kGyroReversed ? -angle : angle) + DriveConstants.kGyroMountingOffsetDeg;
        return Rotation2d.fromDegrees(adjusted);
    }

    private double getGyroAngleDegrees() {
        double angle = m_gyro.getAngle(DriveConstants.kGyroYawAxis);
        return DriveConstants.kGyroAngleIsRotations ? Units.rotationsToDegrees(angle) : angle;
    }
}