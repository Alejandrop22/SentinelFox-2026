package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
	private static final int kIntakeMotorCanId = 55;
	private static final double kReversePercent = -0.6;
	private static final double kFullReversePercent = -1.0;
	private static final double kForwardPercent = 0.4;

	private final SparkMax m_intakeMotor = new SparkMax(kIntakeMotorCanId, MotorType.kBrushless);

	public void intakeReverse() {
		m_intakeMotor.set(kReversePercent);
	}

	public void intakeFullReverse() {
		m_intakeMotor.set(kFullReversePercent);
	}

	public void intakeForward() {
		m_intakeMotor.set(kForwardPercent);
	}

	public void stop() {
		m_intakeMotor.set(0.0);
	}
}
