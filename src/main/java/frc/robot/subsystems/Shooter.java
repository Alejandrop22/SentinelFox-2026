//holaaa
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class Shooter extends SubsystemBase {
	private final SparkMax m_shooterMotor51 = new SparkMax(51, MotorType.kBrushless);
	private final SparkMax m_beltMotor53 = new SparkMax(53, MotorType.kBrushless);
	private final RelativeEncoder m_shooterEncoder;
	private final SparkMaxConfig m_shooterConfig = new SparkMaxConfig();

	// Hard stop override:
	// Cuando esto está activo, el shooter DEBE estar apagado sin importar qué comando lo esté pidiendo.
	private static final String kHardStopKey = "Shooter/HardStop";
	private boolean m_hardStop = false;
	private boolean m_hardStopDashboardInitialized = false;

	// Slew/ramp: evita cambios de putazo (reduce pico de corriente -> menos caída de batería)
	private static final String kRampTimeKey = "Shooter/Ramp/TimeS";
	private boolean m_rampDashboardInitialized = false;
	private double m_rampTimeS = 0.90; // rampa más lenta para reducir pico de pila
	private double m_lastRampUpdateS = Timer.getFPGATimestamp();
	private double m_lastRequestedPercentRaw = 0.0;
	private double m_rampedPercent = 0.0;

	// Soft-start: limita el % durante un arranque para evitar picos de pila.
	private static final double kSoftStartTimeS = 0.20;
	private static final double kSoftStartMaxPercent = 0.70;
	private double m_softStartUntilS = 0.0;
	private boolean m_lastRequestWasZero = true;
	private double m_lastNonIdleRequestS = 0.0;
	private static final double kIdleReturnDelayS = 0.15;

	// Max RPM usado para calcular setpoint (no PID)
	private static final String kMaxRpmKey = "Shooter/MaxRpm";
	private boolean m_maxRpmDashboardInitialized = false;
	private double m_velMaxRpm = 6000.0; // tunable max RPM
	private double m_velocitySetpointRpm = 0.0;

	// Kicker: pequeño boost temporal cuando caen RPM (solo en open-loop)
	private static final String kKickerEnableKey = "Shooter/Kicker/Enable";
	private static final String kKickerBoostKey = "Shooter/Kicker/Boost";
	private static final String kKickerDurationKey = "Shooter/Kicker/DurationS";
	private static final String kKickerDropRpmKey = "Shooter/Kicker/DropRpm";
	private static final String kKickerMinTargetRpmKey = "Shooter/Kicker/MinTargetRpm";
	private boolean m_kickerDashboardInitialized = false;
	private boolean m_kickerEnabled = true;
	private double m_kickerBoostPercent = 0.30;
	private double m_kickerDurationS = 0.45;
	private double m_kickerDropRpm = 50.0;
	private double m_kickerMinTargetRpm = 800.0;
	private double m_kickerUntilS = 0.0;
	private boolean m_kickerActive = false;

	private boolean m_emergencyEnabled = false;
	private static final double kEmergencyPercent = -0.5;
	private static final double kManualShooterPercent = -1.0;
	private static final double kBeltPercent = 0.5;
	private boolean m_assistedActive = false;
	// Idle spin deshabilitado: el shooter SOLO se mueve cuando se ordena por botones/autos.
	// (El ramp sigue activo para subir/bajar suave.)
	private double m_targetPercent = 0.0;
	private String m_lastShooterCommand = "none";
	// Idle spin habilitado para mantener -0.15 cuando no hay comando
	private boolean m_idleSpinEnabled = true;
	@SuppressWarnings("unused")
	private double m_lastIdleAppliedPercent = 0.0;
	private String m_requestSource = "none";
	private boolean m_idleRequestActive = false;
	private static final double kIdlePercent = -0.15;

	// Control por porcentaje con compensación por bus voltage.
	// Ajustamos el % real según la pila para mantener la misma respuesta.
	private static final double kNominalVoltage = 12.0;
	private double m_commandedVolts = 0.0;
	private double m_compensatedPercent = 0.0;

	// Campos legacy (idle/kick) ya no se usan, pero los dejamos por estabilidad del archivo.
	@SuppressWarnings("unused")
	private static final int kStartupKickCycles = 8;
	@SuppressWarnings("unused")
	private static final double kStartupKickPercent = -0.70;
	@SuppressWarnings("unused")
	private int m_startupKickRemaining = 0;
	@SuppressWarnings("unused")
	private double m_lastRequestedPercent = 0.0;

	public Shooter() {
		m_shooterConfig.smartCurrentLimit(60);
		// Sin compensación interna por voltaje: usamos % + compensación manual.
		m_shooterConfig.idleMode(IdleMode.kCoast);
		m_shooterMotor51.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_shooterEncoder = m_shooterMotor51.getEncoder();
	}

	private void applyHardStop(String reason) {
		m_assistedActive = false;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_lastRequestedPercentRaw = 0.0;
		m_targetPercent = 0.0;
		m_rampedPercent = 0.0;
		m_commandedVolts = 0.0;
		m_compensatedPercent = 0.0;
		m_requestSource = "hardStop";
		m_lastShooterCommand = "HARD_STOP: " + reason;
		m_shooterMotor51.set(0.0);
	}

	private double applyRamp(double requestedPercent) {
		// Clamp a rango válido
		double req = MathUtil.clamp(requestedPercent, -1.0, 1.0);
		m_lastRequestedPercentRaw = req;

		double now = Timer.getFPGATimestamp();
		double dt = now - m_lastRampUpdateS;
		// Si dt raro (primer ciclo / lag) evita brincos.
		if (dt < 0.0 || dt > 0.25) {
			dt = 0.02;
		}
		m_lastRampUpdateS = now;

		// rampTimeS = tiempo en pasar de 0 a 1.0 (full-scale)
		double rampTime = Math.max(0.02, m_rampTimeS);
		double maxDelta = dt / rampTime;
		m_rampedPercent = MathUtil.clamp(
			m_rampedPercent + MathUtil.clamp(req - m_rampedPercent, -maxDelta, maxDelta),
			-1.0,
			1.0);
		return m_rampedPercent;
	}

	private void setShooterPercentInternal(double requestedPercent) {
		if (m_hardStop) {
			applyHardStop("blocked setShooterPercentInternal(" + requestedPercent + ")");
			return;
		}

		boolean requestIsZero = Math.abs(requestedPercent) < 1e-6;
		if (!m_idleRequestActive) {
			if (!requestIsZero && m_lastRequestWasZero) {
				m_softStartUntilS = Timer.getFPGATimestamp() + kSoftStartTimeS;
			}
			m_lastRequestWasZero = requestIsZero;
			if (!requestIsZero) {
				m_lastNonIdleRequestS = Timer.getFPGATimestamp();
			}
		}

		double effectiveRequestedPercent = requestedPercent;
		if (!m_idleRequestActive && Timer.getFPGATimestamp() < m_softStartUntilS) {
			effectiveRequestedPercent = Math.copySign(
				Math.min(Math.abs(requestedPercent), kSoftStartMaxPercent),
				requestedPercent);
		}
		// Ramp (subida y bajada). Esto reemplaza el "kick" como método principal anti-pico.
		// (Dejamos variables de kick por compatibilidad, pero ya no se usan aquí.)
		double rampedPercent = applyRamp(effectiveRequestedPercent);
		boolean requestIsOn = Math.abs(rampedPercent) > 1e-6;
		m_lastRequestedPercent = rampedPercent;

		if (m_emergencyEnabled) {
			m_lastShooterCommand = "percent_blocked_by_emergency";
			m_targetPercent = kEmergencyPercent;
			m_velocitySetpointRpm = kEmergencyPercent * m_velMaxRpm;
			m_compensatedPercent = kEmergencyPercent;
			m_shooterMotor51.set(m_compensatedPercent);
			return;
		}

		double appliedPercent = rampedPercent;
		m_targetPercent = requestedPercent; // target = lo que pidió arriba; applied = lo que realmente sale (ramped)
		m_velocitySetpointRpm = Math.copySign(
			MathUtil.clamp(Math.abs(appliedPercent), 0.0, 1.0) * m_velMaxRpm,
			appliedPercent);
		double busVoltage = Math.max(6.0, m_shooterMotor51.getBusVoltage());
		m_compensatedPercent = MathUtil.clamp(appliedPercent * (kNominalVoltage / busVoltage), -1.0, 1.0);
		m_commandedVolts = busVoltage * m_shooterMotor51.getAppliedOutput();
		m_requestSource = m_idleRequestActive
			? "idle"
			: (m_assistedActive ? "assisted" : (requestIsOn ? "manual" : "stop"));
		m_lastShooterCommand = String.format(
			"percentCmd(%.3f) target(%.3f) appliedRamped(%.3f) rampTimeS(%.2f)",
			m_compensatedPercent,
			requestedPercent,
			appliedPercent,
			m_rampTimeS);
		double now = Timer.getFPGATimestamp();
		double appliedPercentOpenLoop = m_compensatedPercent;
		double setpointAbs = Math.abs(m_velocitySetpointRpm);
		double actualAbs = Math.abs(m_shooterEncoder.getVelocity());
		if (m_kickerEnabled
				&& !m_idleRequestActive
				&& setpointAbs >= m_kickerMinTargetRpm
				&& (setpointAbs - actualAbs) >= m_kickerDropRpm) {
			m_kickerUntilS = now + m_kickerDurationS;
		}
		m_kickerActive = now < m_kickerUntilS;
		if (m_kickerActive) {
			appliedPercentOpenLoop = MathUtil.clamp(
				appliedPercentOpenLoop + Math.copySign(m_kickerBoostPercent, appliedPercentOpenLoop),
				-1.0,
				1.0);
		}
		m_compensatedPercent = appliedPercentOpenLoop;
		m_shooterMotor51.set(m_compensatedPercent);
	}

	/**
	 * Set shooter percent WITHOUT startup-kick logic.
	 *
	 * <p>Usado para el idle spin, para que no se convierta en un "kick" fuerte cada vez que
	 * se prende/apaga desde Shuffleboard.
	 */
	// setShooterPercentNoKick eliminado: era sólo para idle spin.

	public void startManualShooter() {
		m_assistedActive = false;
		m_requestSource = "manual";
		setShooterPercentInternal(kManualShooterPercent);
	}

	public void setAssistedShooterPercent(double percent) {
		m_assistedActive = true;
		m_requestSource = "assisted";
		setShooterPercentInternal(percent);
	}

	/**
	 * Quita el estado de "assisted" sin necesariamente mandar 0V.
	 *
	 * <p>Esto es util cuando el comando que controlaba el shooter termina (por ejemplo, al soltar RT)
	 * y queremos que el subsistema vuelva a su logica normal (idle spin / manual) sin dejar
	 * el flag de assisted activo.
	 */
	public void clearAssistedRequest() {
		m_assistedActive = false;
		if (m_requestSource.equals("assisted")) {
			m_requestSource = "none";
		}
	}

	public void stopManualShooter() {
		m_assistedActive = false;
		m_requestSource = "stop";
		// Mantener comportamiento con rampa: apaga suave (por ejemplo 0.5s) para evitar picos.
		setShooterPercentInternal(0.0);
	}

	/**
	 * Termina un comando de disparo y regresa al idle spin (si esta habilitado).
	 *
	 * <p>Esto se usa al soltar RT: queremos que el shooter SIEMPRE vuelva a -0.15
	 * (con rampa) en vez de quedarse en el ultimo setpoint alto.
	 */
	public void endShootAndReturnToIdle() {
		// Volver a idle (con rampa) si está habilitado.
		m_assistedActive = false;
		m_requestSource = "return_to_idle";
		if (m_idleSpinEnabled) {
			m_idleRequestActive = true;
			setShooterPercentInternal(kIdlePercent);
			m_idleRequestActive = false;
		} else {
			setShooterPercentInternal(0.0);
		}
	}

	/**
	 * Habilita/deshabilita el "idle spin". Por default está habilitado.
	 *
	 * <p>Cuando está habilitado, el shooter mantiene un mínimo de -0.15 mientras el robot
	 * esté ENABLED y no haya un comando activo que lo sobre-escriba.
	 */
	public void setIdleSpinEnabled(boolean enabled) {
		m_idleSpinEnabled = enabled;
	}

	public void stop() {
		m_assistedActive = false;
		m_lastShooterCommand = "stop";
		m_requestSource = "stop";
		m_targetPercent = 0.0;
		m_commandedVolts = 0.0;
		m_velocitySetpointRpm = 0.0;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_lastRequestedPercentRaw = 0.0;
		m_rampedPercent = 0.0;
		m_shooterMotor51.set(0.0);
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
		if (m_hardStop) {
			applyHardStop("blocked toggleEmergencyShooter");
			return;
		}
		m_emergencyEnabled = !m_emergencyEnabled;
		m_targetPercent = 0.0;
		m_commandedVolts = 0.0;
		m_velocitySetpointRpm = 0.0;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_shooterMotor51.set(m_emergencyEnabled ? kEmergencyPercent : 0.0);
	}

	public boolean isEmergencyEnabled() {
		return m_emergencyEnabled;
	}

	public void disableEmergency() {
		m_emergencyEnabled = false;
		m_lastShooterCommand = "disableEmergency";
		m_requestSource = "stop";
		m_targetPercent = 0.0;
		m_commandedVolts = 0.0;
		m_velocitySetpointRpm = 0.0;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		// El usuario pidió que al desactivar emergencia se apague el motor.
		m_shooterMotor51.set(0.0);
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
		// Ramp time tuneable
		if (!m_rampDashboardInitialized) {
			SmartDashboard.putNumber(kRampTimeKey, m_rampTimeS);
			m_rampDashboardInitialized = true;
		}
		m_rampTimeS = SmartDashboard.getNumber(kRampTimeKey, m_rampTimeS);
		m_rampTimeS = MathUtil.clamp(m_rampTimeS, 0.05, 2.0);

		if (!m_maxRpmDashboardInitialized) {
			SmartDashboard.putNumber(kMaxRpmKey, m_velMaxRpm);
			m_maxRpmDashboardInitialized = true;
		}
		if (!m_kickerDashboardInitialized) {
			SmartDashboard.putBoolean(kKickerEnableKey, m_kickerEnabled);
			SmartDashboard.putNumber(kKickerBoostKey, m_kickerBoostPercent);
			SmartDashboard.putNumber(kKickerDurationKey, m_kickerDurationS);
			SmartDashboard.putNumber(kKickerDropRpmKey, m_kickerDropRpm);
			SmartDashboard.putNumber(kKickerMinTargetRpmKey, m_kickerMinTargetRpm);
			m_kickerDashboardInitialized = true;
		}
		m_velMaxRpm = SmartDashboard.getNumber(kMaxRpmKey, m_velMaxRpm);
		m_kickerEnabled = SmartDashboard.getBoolean(kKickerEnableKey, m_kickerEnabled);
		m_kickerBoostPercent = SmartDashboard.getNumber(kKickerBoostKey, m_kickerBoostPercent);
		m_kickerDurationS = SmartDashboard.getNumber(kKickerDurationKey, m_kickerDurationS);
		m_kickerDropRpm = SmartDashboard.getNumber(kKickerDropRpmKey, m_kickerDropRpm);
		m_kickerMinTargetRpm = SmartDashboard.getNumber(kKickerMinTargetRpmKey, m_kickerMinTargetRpm);

		// Hard stop desde Shuffleboard:
		//  - publicar default una sola vez
		//  - luego sólo leer
		if (!m_hardStopDashboardInitialized) {
			SmartDashboard.putBoolean(kHardStopKey, m_hardStop);
			m_hardStopDashboardInitialized = true;
		}
		m_hardStop = SmartDashboard.getBoolean(kHardStopKey, m_hardStop);
		if (m_hardStop) {
			applyHardStop("periodic");
		}

		// Si ya no se está pidiendo shooter (target=0) pero aún quedó voltaje alto por la rampa,
		// sigue aplicando la rampa hacia 0 en cada ciclo para asegurar que realmente se apague.
		if (!m_hardStop
				&& !m_emergencyEnabled
				&& !m_assistedActive
				&& Math.abs(m_targetPercent) < 1e-6
				&& Math.abs(m_rampedPercent) > 1e-4) {
			setShooterPercentInternal(0.0);
		}

		// Idle spin: mantener -0.15 cuando no hay comando activo (con pequeño delay)
		double now = Timer.getFPGATimestamp();
		if (!m_hardStop
				&& !m_emergencyEnabled
				&& !m_assistedActive
				&& m_idleSpinEnabled
				&& DriverStation.isEnabled()
				&& (now - m_lastNonIdleRequestS) >= kIdleReturnDelayS) {
			m_idleRequestActive = true;
			setShooterPercentInternal(kIdlePercent);
			m_idleRequestActive = false;
			m_lastIdleAppliedPercent = kIdlePercent;
		} else {
			m_lastIdleAppliedPercent = 0.0;
		}

		// En modo % con compensación no necesitamos periodic para controlar, sólo para telemetría.
		SmartDashboard.putBoolean("Shooter/EmergencyEnabled", m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/AssistedEnabled", m_assistedActive && !m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/IdleSpinEnabled", m_idleSpinEnabled);
		SmartDashboard.putBoolean("Shooter/HardStopActive", m_hardStop);
		SmartDashboard.putNumber("Shooter/Ramp/TimeS", m_rampTimeS);
		SmartDashboard.putNumber("Shooter/RequestedPercentRaw", m_lastRequestedPercentRaw);
		SmartDashboard.putNumber("Shooter/AppliedPercentRamped", m_rampedPercent);
		SmartDashboard.putNumber("Shooter/IdleSpinPercent", kIdlePercent);
		SmartDashboard.putNumber("Shooter/IdleSpin/AppliedPercent", m_lastIdleAppliedPercent);
		SmartDashboard.putNumber("Shooter/IdleSpin/AppliedVolts", m_shooterMotor51.getBusVoltage() * m_lastIdleAppliedPercent);
		SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterEncoder.getVelocity());
		SmartDashboard.putNumber("Shooter/VelocityRPMAbs", Math.abs(m_shooterEncoder.getVelocity()));
		SmartDashboard.putNumber("Shooter/TargetPercent", m_targetPercent);
		SmartDashboard.putNumber("Shooter/TargetVolts", kNominalVoltage * m_targetPercent);
		SmartDashboard.putNumber("Shooter/CommandedVolts", m_commandedVolts);
		SmartDashboard.putNumber("Shooter/CompensatedPercent", m_compensatedPercent);
		SmartDashboard.putNumber("Shooter/VelocitySetpointRpm", m_velocitySetpointRpm);
		SmartDashboard.putNumber("Shooter/AppliedPercent", m_shooterMotor51.getAppliedOutput());
		SmartDashboard.putNumber("Shooter/BusVoltage", m_shooterMotor51.getBusVoltage());
		SmartDashboard.putNumber("Shooter/AppliedVoltsApprox", m_shooterMotor51.getBusVoltage() * m_shooterMotor51.getAppliedOutput());
		SmartDashboard.putNumber("Shooter/OutputCurrentA", m_shooterMotor51.getOutputCurrent());
		SmartDashboard.putNumber("Shooter/MaxRpm", m_velMaxRpm);
		SmartDashboard.putBoolean("Shooter/Kicker/Active", m_kickerActive);
		SmartDashboard.putBoolean("Shooter/Kicker/Enable", m_kickerEnabled);
		SmartDashboard.putNumber("Shooter/Kicker/Boost", m_kickerBoostPercent);
		SmartDashboard.putNumber("Shooter/Kicker/DurationS", m_kickerDurationS);
		SmartDashboard.putNumber("Shooter/Kicker/DropRpm", m_kickerDropRpm);
		SmartDashboard.putNumber("Shooter/Kicker/MinTargetRpm", m_kickerMinTargetRpm);
		SmartDashboard.putString("Shooter/LastCommand", m_lastShooterCommand);
		SmartDashboard.putString("Shooter/RequestSource", m_requestSource);
	}
}
