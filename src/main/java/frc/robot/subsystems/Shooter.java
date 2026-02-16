package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {
	private final SparkMax m_shooterMotor51 = new SparkMax(51, MotorType.kBrushless);
	private final SparkMax m_beltMotor53 = new SparkMax(53, MotorType.kBrushless);
	private final RelativeEncoder m_shooterEncoder;

	private boolean m_emergencyEnabled = false;
	private static final double kEmergencyPercent = -0.5;
	private boolean m_assistedActive = false;

	public Shooter() {
		m_shooterEncoder = m_shooterMotor51.getEncoder();
	}

	public void startManualShooter() {
		m_assistedActive = false;
		if (m_emergencyEnabled) {
			m_shooterMotor51.set(kEmergencyPercent);
			return;
		}
		m_shooterMotor51.set(-0.8);
	}

	public void setAssistedShooterPercent(double percent) {
		m_assistedActive = true;
		if (m_emergencyEnabled) {
			m_shooterMotor51.set(kEmergencyPercent);
			return;
		}
		m_shooterMotor51.set(percent);
	}

	public void stopManualShooter() {
		m_assistedActive = false;
		if (m_emergencyEnabled) {
			m_shooterMotor51.set(kEmergencyPercent);
			return;
		}
		m_shooterMotor51.set(0.0);
	}

	public void toggleEmergencyShooter() {
		m_emergencyEnabled = !m_emergencyEnabled;
		m_shooterMotor51.set(m_emergencyEnabled ? kEmergencyPercent : 0.0);
	}

	public boolean isEmergencyEnabled() {
		return m_emergencyEnabled;
	}

	public void disableEmergency() {
		m_emergencyEnabled = false;
	}

	public boolean isAssistedActive() {
		return m_assistedActive;
	}

	public void startBelt() {
		m_beltMotor53.set(0.5);
	}

	public void stopBelt() {
		m_beltMotor53.set(0.0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Shooter/EmergencyEnabled", m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/AssistedEnabled", m_assistedActive && !m_emergencyEnabled);
		SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterEncoder.getVelocity());
		SmartDashboard.putNumber("Shooter/AppliedPercent", m_shooterMotor51.getAppliedOutput());
	}
}
