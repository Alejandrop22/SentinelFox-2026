package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Align {
	private static final double kTargetDistanceM = 1.0;
	private static final double kMaxSpeed = 0.6;
	private static final double kMaxStrafe = 0.6;
	private static final double kMaxRot = 0.6;

	private static final double kPx = 0.9;
	private static final double kPy = 0.9;
	private static final double kPtheta = 0.02; // deg -> rot cmd

	private static final double kDistTolM = 0.05;
	private static final double kYawTolDeg = 2.0;
	private static final double kLatTolM = 0.05;

	private Align() {}

	public static Command createCommand(DriveSubsystem drive, Camara camara) {
		return new RunCommand(() -> executeAlign(drive, camara), drive);
	}

	public static Command createOneShotCommand(DriveSubsystem drive, Camara camara) {
		return new RunCommand(() -> executeAlign(drive, camara), drive)
			.until(() -> isAtGoal(camara))
			.andThen(stopCommand(drive));
	}

	public static Command stopCommand(DriveSubsystem drive) {
		return new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0, false), drive);
	}

	public static void publishDefaults() {
		SmartDashboard.putNumber("Align/YawDeg", 0.0);
		SmartDashboard.putNumber("Align/DistanceM", 0.0);
		SmartDashboard.putNumber("Align/XErrorM", 0.0);
		SmartDashboard.putNumber("Align/YErrorM", 0.0);
		SmartDashboard.putBoolean("Align/AtGoal", false);
		SmartDashboard.putString("Align/State", "IDLE");
	}

	private static void executeAlign(DriveSubsystem drive, Camara camara) {
		if (!camara.hasAutoAimTag()) {
			drive.drive(0.0, 0.0, 0.0, false);
			SmartDashboard.putString("Align/State", "NO_TAG");
			return;
		}

		double yawDeg = camara.getAutoAimYawDeg();
		double distance = camara.getAutoAimDistanceM();
		double yawRad = Math.toRadians(yawDeg);

		// Approximate lateral offset using yaw and distance.
		double xMeters = distance * Math.cos(yawRad);
		double yMeters = distance * Math.sin(yawRad);

		double xError = xMeters - kTargetDistanceM;
		double yError = yMeters;
		double rotError = yawDeg;

		double xCmd = MathUtil.clamp(kPx * xError, -kMaxSpeed, kMaxSpeed);
		double yCmd = MathUtil.clamp(kPy * yError, -kMaxStrafe, kMaxStrafe);
		double rotCmd = MathUtil.clamp(kPtheta * rotError, -kMaxRot, kMaxRot);

		drive.drive(xCmd, yCmd, rotCmd, false);

		SmartDashboard.putNumber("Align/YawDeg", yawDeg);
		SmartDashboard.putNumber("Align/DistanceM", distance);
		SmartDashboard.putNumber("Align/XErrorM", xError);
		SmartDashboard.putNumber("Align/YErrorM", yError);
		SmartDashboard.putNumber("Align/XCmd", xCmd);
		SmartDashboard.putNumber("Align/YCmd", yCmd);
		SmartDashboard.putNumber("Align/RotCmd", rotCmd);
		SmartDashboard.putString("Align/State", "RUNNING");

		boolean done = Math.abs(xError) < kDistTolM
			&& Math.abs(yError) < kLatTolM
			&& Math.abs(yawDeg) < kYawTolDeg;
		SmartDashboard.putBoolean("Align/AtGoal", done);
	}

	private static boolean isAtGoal(Camara camara) {
		if (!camara.hasAutoAimTag()) {
			return false;
		}
		double yawDeg = camara.getAutoAimYawDeg();
		double distance = camara.getAutoAimDistanceM();
		double yawRad = Math.toRadians(yawDeg);
		double xMeters = distance * Math.cos(yawRad);
		double yMeters = distance * Math.sin(yawRad);

		return Math.abs(xMeters - kTargetDistanceM) < kDistTolM
			&& Math.abs(yMeters) < kLatTolM
			&& Math.abs(yawDeg) < kYawTolDeg;
	}
}
