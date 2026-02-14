package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Belt motor moved to CAN SparkMax (ID 53)
public class Shooter extends SubsystemBase {
	// Shooter motor (CAN 51) and conveyor/belt motor (CAN 53)
	private final SparkMax m_shooterMotor = new SparkMax(51, MotorType.kBrushless);
	private final SparkMax m_beltMotor53 = new SparkMax(53, MotorType.kBrushless);

	/** Start launcher at default preset (-70%). */
	public void startLauncher() {
		m_shooterMotor.set(-0.7); // 70% hacia atras
	}

	/** Stop the launcher motor. */
	public void stopLauncher() {
		m_shooterMotor.set(0.0);
	}

	/**
	 * Set the launcher motor percent output directly.
	 * Use a negative value if your hardware expects that direction to shoot.
	 * @param percent -1.0..1.0
	 */
	public void setLauncherPercent(double percent) {
		m_shooterMotor.set(percent);
	}

	public void startBelt() {
		m_beltMotor53.set(0.5); // 50% hacia adelante
	}

	public void stopBelt() {
		m_beltMotor53.set(0.0);
	}
}
