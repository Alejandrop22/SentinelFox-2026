package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
	private final SparkMax m_intakeMotor = new SparkMax(55, MotorType.kBrushless);

	public void intakeReverse() {
	m_intakeMotor.set(0.6);
	}

	public void intakeFullReverse() {
	m_intakeMotor.set(1.0);
	}

	public void intakeForward() {
	m_intakeMotor.set(-0.4);
	}

	public void stop() {
	m_intakeMotor.set(0.0);
	}
}
