package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

// Belt motor moved to CAN SparkMax (ID 53)
public class Shooter extends SubsystemBase {
	// Manual shooter motor (CAN 51) and conveyor/belt motor (CAN 53)
	private final SparkMax m_shooterMotor51 = new SparkMax(51, MotorType.kBrushless);
	private final SparkMax m_beltMotor53 = new SparkMax(53, MotorType.kBrushless);
	private final RelativeEncoder m_shooterEncoder;

	// Emergency toggle: left stick click sets shooter to a safe-ish constant speed.
	private boolean m_emergencyEnabled = false;
	private static final double kEmergencyPercent = -0.5;
	private boolean m_assistedActive = false;

	public Shooter() {
		m_shooterEncoder = m_shooterMotor51.getEncoder();
	}

	/** Manual shooter: always run at -0.8 while held. */
	public void startManualShooter() {
		m_assistedActive = false;
		// If emergency is enabled, don't override it.
		if (m_emergencyEnabled) {
			m_shooterMotor51.set(kEmergencyPercent);
			return;
		}
		m_shooterMotor51.set(-0.8);
	}

	/** Apply assisted percent output to shooter motor (CAN 51). */
	public void setAssistedShooterPercent(double percent) {
		m_assistedActive = true;
		// Emergency has top priority; don't override it.
		if (m_emergencyEnabled) {
			m_shooterMotor51.set(kEmergencyPercent);
			return;
		}
		m_shooterMotor51.set(percent);
	}

	public void stopManualShooter() {
		m_assistedActive = false;
		// If emergency is enabled, don't override it.
		if (m_emergencyEnabled) {
			m_shooterMotor51.set(kEmergencyPercent);
			return;
		}
		m_shooterMotor51.set(0.0);
	}

	/** Toggle emergency shooter mode (-0.5). */
	public void toggleEmergencyShooter() {
		m_emergencyEnabled = !m_emergencyEnabled;
		m_shooterMotor51.set(m_emergencyEnabled ? kEmergencyPercent : 0.0);
	}

	public boolean isEmergencyEnabled() {
		return m_emergencyEnabled;
	}

	/** Force emergency off (used when other modes need full control). */
	public void disableEmergency() {
		m_emergencyEnabled = false;
	}

	public boolean isAssistedActive() {
		return m_assistedActive;
	}

	public void startBelt() {
		m_beltMotor53.set(0.5); // 50% hacia adelante
	}

	public void stopBelt() {
		m_beltMotor53.set(0.0);
	}

	@Override
	public void periodic() {
		// Dashboard: mode booleans
		SmartDashboard.putBoolean("Shooter/EmergencyEnabled", m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/AssistedEnabled", m_assistedActive && !m_emergencyEnabled);

		// Velocity: if encoder isn't configured for RPM scaling, this will still be consistent
		// enough to show movement; teams can later apply conversion if desired.
		SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterEncoder.getVelocity());
		SmartDashboard.putNumber("Shooter/AppliedPercent", m_shooterMotor51.getAppliedOutput());
	}
}
