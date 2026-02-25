package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.Set;

public class DriveSubsystem_2 extends SubsystemBase {
	private boolean m_rotationOverrideActive = false;
	private double m_rotationOverrideValue = 0.0;
	private final Camara m_camara;
	private boolean m_autoAimEnabled = false;
	private final PIDController m_autoAimPID = new PIDController(0.018, 0.0, 0.001);
	private static final double kAutoAimToleranceDeg = 1.0;
	private static final double kAutoAimMaxRotCmd = 0.6;
	private Double m_autoAimTargetHeadingDeg = null;

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

	public DriveSubsystem_2(Camara camara) {
		m_camara = camara;
		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
		m_gyro.calibrate();
		m_gyro.reset();

			m_autoAimPID.enableContinuousInput(-180.0, 180.0);
			m_autoAimPID.setTolerance(kAutoAimToleranceDeg);
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
		SmartDashboard.putNumber("Drive2/HeadingDeg", getHeading());
		SmartDashboard.putNumber("Drive2/TurnRateDps", getTurnRate());
		SmartDashboard.putNumber("Drive2/PoseX", pose.getX());
		SmartDashboard.putNumber("Drive2/PoseY", pose.getY());
		SmartDashboard.putNumber("Drive2/PoseDeg", pose.getRotation().getDegrees());
	}

	public synchronized void setAutoAimEnabled(boolean enabled) {
		m_autoAimEnabled = enabled;
		if (!enabled) {
			clearRotationOverride();
				m_autoAimTargetHeadingDeg = null;
		}
	}

	public synchronized boolean isAutoAimEnabled() {
		return m_autoAimEnabled;
	}

	private void updateAutoAimRotation() {
		if (!m_autoAimEnabled) {
			SmartDashboard.putBoolean("AutoAim2/Enabled", false);
				m_autoAimTargetHeadingDeg = null;
			return;
		}

		SmartDashboard.putBoolean("AutoAim2/Enabled", true);

		if (!m_camara.hasAutoAimTag()) {
			setRotationOverride(0.0);
				m_autoAimTargetHeadingDeg = null;
				SmartDashboard.putBoolean("AutoAim2/TrackingTag", false);
			return;
		}

			SmartDashboard.putBoolean("AutoAim2/TrackingTag", true);

			double currentHeading = getHeading();
			double tx = m_camara.getAutoAimYawDeg();

			m_autoAimTargetHeadingDeg = MathUtil.inputModulus(currentHeading - tx, -180.0, 180.0);

			double rotCmd = m_autoAimPID.calculate(currentHeading, m_autoAimTargetHeadingDeg);
			rotCmd = MathUtil.clamp(rotCmd, -kAutoAimMaxRotCmd, kAutoAimMaxRotCmd);

			setRotationOverride(rotCmd);

			SmartDashboard.putNumber("AutoAim2/CurrentHeading", currentHeading);
			SmartDashboard.putNumber("AutoAim2/TargetHeading", m_autoAimTargetHeadingDeg);
			SmartDashboard.putNumber("AutoAim2/tx", tx);
			SmartDashboard.putNumber("AutoAim2/RotCmd", rotCmd);
			SmartDashboard.putBoolean("AutoAim2/AtSetpoint", m_autoAimPID.atSetpoint());
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

		double rotToUse = getRotationForDrive(rot);
		double rotDelivered = rotToUse * DriveConstants.kMaxAngularSpeed;

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

	private synchronized double getRotationForDrive(double requestedRot) {
		if (m_rotationOverrideActive) return m_rotationOverrideValue;
		return requestedRot;
	}

	private synchronized void setRotationOverride(double rotCmd) {
		m_rotationOverrideActive = true;
		m_rotationOverrideValue = rotCmd;
	}

	private synchronized void clearRotationOverride() {
		m_rotationOverrideActive = false;
		m_rotationOverrideValue = 0.0;
	}

	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
	}

	public void zeroHeading() {
		m_gyro.reset();
	}

	public double getHeading() {
		return MathUtil.inputModulus(getGyroAngleDegrees(), -180.0, 180.0);
	}

	public double getTurnRate() {
		return m_gyro.getRate();
	}

	private Rotation2d getGyroRotation() {
		return Rotation2d.fromDegrees(getGyroAngleDegrees());
	}

	private double getGyroAngleDegrees() {
		return m_gyro.getAngle();
	}

	public synchronized ChassisSpeeds getLastChassisSpeeds() {
		return m_lastChassisSpeeds;
	}

	public void stopModules() {
			drive(0.0, 0.0, 0.0, false);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0], true);
		m_frontRight.setDesiredState(desiredStates[1], true);
		m_rearLeft.setDesiredState(desiredStates[2], true);
		m_rearRight.setDesiredState(desiredStates[3], true);
	}

	public Command applyRequest(java.util.function.Supplier<ChassisSpeeds> speedsSupplier) {
		return run(() -> {
			ChassisSpeeds spd = speedsSupplier.get();
			drive(
					spd.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
					spd.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
					spd.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
					false);
		});
	}

		public void logTurningEncoders() {
		}

	public static double metersToFeet(double meters) {
		return Units.metersToFeet(meters);
	}

	public Set<MAXSwerveModule> getModules() {
		return Set.of(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
	}
}
