package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Shooter extends SubsystemBase {
	private final SparkMax m_shooterMotor51 = new SparkMax(51, MotorType.kBrushless);
	private final SparkMax m_beltMotor53 = new SparkMax(53, MotorType.kBrushless);
	private final RelativeEncoder m_shooterEncoder;
	private final SparkMaxConfig m_shooterConfig = new SparkMaxConfig();

	private boolean m_emergencyEnabled = false;
	private static final double kEmergencyPercent = -0.5;
	private static final double kManualShooterPercent = -1.0;
	private static final double kBeltPercent = 0.5;
	private boolean m_assistedActive = false;
	private double m_targetPercent = 0.0;
	private String m_lastShooterCommand = "none";

	// Control por voltaje: percent (-1..1) se convierte a volts = 12*percent.
	// Esto ayuda a que el shooter sea menos sensible a la pila (vs .set(percent) puro).
	private static final double kNominalVoltage = 12.0;
	private double m_commandedVolts = 0.0;

	// "Kick" de arranque para vencer fricción estática (evita que arranque feo o titubee).
	// 150ms ~= 8 ciclos de periodic (20ms.)
	private static final int kStartupKickCycles = 8;
	private static final double kStartupKickPercent = -0.70;
	private int m_startupKickRemaining = 0;
	private double m_lastRequestedPercent = 0.0;

	public Shooter() {
		m_shooterConfig.smartCurrentLimit(60);
		m_shooterConfig.voltageCompensation(kNominalVoltage);
		m_shooterConfig.idleMode(IdleMode.kCoast);
		m_shooterMotor51.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_shooterEncoder = m_shooterMotor51.getEncoder();
	}

	private void setShooterPercentInternal(double requestedPercent) {
		// Detectar transición de "apagado" -> "prender" para armar el kick
		boolean requestIsOn = Math.abs(requestedPercent) > 1e-6;
		boolean wasOff = Math.abs(m_lastRequestedPercent) <= 1e-6;
		if (requestIsOn && wasOff && !m_emergencyEnabled) {
			m_startupKickRemaining = kStartupKickCycles;
		}
		m_lastRequestedPercent = requestedPercent;

		double targetVolts;
		if (m_emergencyEnabled) {
			m_lastShooterCommand = "percent_blocked_by_emergency";
			m_targetPercent = kEmergencyPercent;
			targetVolts = kNominalVoltage * kEmergencyPercent;
			m_commandedVolts = targetVolts;
			m_shooterMotor51.setVoltage(targetVolts);
			return;
		}

		// Aplicar kick por unos ciclos al arranque, luego mantener el percent pedido
		double appliedPercent = requestedPercent;
		if (m_startupKickRemaining > 0 && requestIsOn) {
			m_startupKickRemaining--;
			// Mantener el signo del request; el valor por default es negativo (forward)
			appliedPercent = Math.copySign(Math.abs(kStartupKickPercent), requestedPercent);
		}

		m_targetPercent = requestedPercent;
		targetVolts = kNominalVoltage * appliedPercent;
		m_commandedVolts = targetVolts;
		m_lastShooterCommand = String.format(
			"voltageCmd(%.2fV) from percent(%.3f) appliedPercent(%.3f) kickRemain(%d)",
			targetVolts,
			requestedPercent,
			appliedPercent,
			m_startupKickRemaining);
		m_shooterMotor51.setVoltage(targetVolts);
	}

	public void startManualShooter() {
		m_assistedActive = false;
		setShooterPercentInternal(kManualShooterPercent);
	}

	public void setAssistedShooterPercent(double percent) {
		m_assistedActive = true;
		setShooterPercentInternal(percent);
	}

	public void stopManualShooter() {
		m_assistedActive = false;
		setShooterPercentInternal(0.0);
	}

	public void stop() {
		m_assistedActive = false;
		m_lastShooterCommand = "stop";
		m_targetPercent = 0.0;
		m_commandedVolts = 0.0;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_shooterMotor51.setVoltage(0.0);
	}

	/**
	 * Modo de prueba: manda % directo (open-loop), sin PID.
	 * Útil para aislar si el problema es de control cerrado o de hardware/cableado.
	 */
	public void debugSetShooterPercent(double percent) {
		m_assistedActive = false;
		setShooterPercentInternal(percent);
	}

	public void toggleEmergencyShooter() {
		m_emergencyEnabled = !m_emergencyEnabled;
		m_targetPercent = 0.0;
		m_commandedVolts = 0.0;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_shooterMotor51.setVoltage(m_emergencyEnabled ? (kNominalVoltage * kEmergencyPercent) : 0.0);
	}

	public boolean isEmergencyEnabled() {
		return m_emergencyEnabled;
	}

	public void disableEmergency() {
		m_emergencyEnabled = false;
		m_lastShooterCommand = "disableEmergency";
		m_targetPercent = 0.0;
		m_commandedVolts = 0.0;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		// El usuario pidió que al desactivar emergencia se apague el motor.
		m_shooterMotor51.setVoltage(0.0);
	}

	public double getTargetPercent() {
		return m_targetPercent;
	}

	public boolean isAssistedActive() {
		return m_assistedActive;
	}

	public void startBelt() {
		m_beltMotor53.set(kBeltPercent);
	}

	public void reverseBelt() {
		m_beltMotor53.set(-kBeltPercent);
	}

	public void stopBelt() {
		m_beltMotor53.set(0.0);
	}

	@Override
	public void periodic() {
		// En modo voltaje NO necesitamos periodic para controlar, sólo para telemetría.
		SmartDashboard.putBoolean("Shooter/EmergencyEnabled", m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/AssistedEnabled", m_assistedActive && !m_emergencyEnabled);
		SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterEncoder.getVelocity());
		SmartDashboard.putNumber("Shooter/VelocityRPMAbs", Math.abs(m_shooterEncoder.getVelocity()));
		SmartDashboard.putNumber("Shooter/TargetPercent", m_targetPercent);
		SmartDashboard.putNumber("Shooter/TargetVolts", kNominalVoltage * m_targetPercent);
		SmartDashboard.putNumber("Shooter/CommandedVolts", m_commandedVolts);
		SmartDashboard.putNumber("Shooter/AppliedPercent", m_shooterMotor51.getAppliedOutput());
		SmartDashboard.putNumber("Shooter/BusVoltage", m_shooterMotor51.getBusVoltage());
		SmartDashboard.putNumber("Shooter/AppliedVoltsApprox", m_shooterMotor51.getBusVoltage() * m_shooterMotor51.getAppliedOutput());
		SmartDashboard.putNumber("Shooter/OutputCurrentA", m_shooterMotor51.getOutputCurrent());
		SmartDashboard.putString("Shooter/LastCommand", m_lastShooterCommand);
	}
}
