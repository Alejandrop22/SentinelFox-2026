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
	private double m_rampTimeS = 0.50; // lo que pediste
	private double m_lastRampUpdateS = Timer.getFPGATimestamp();
	private double m_lastRequestedPercentRaw = 0.0;
	private double m_rampedPercent = 0.0;

	private boolean m_emergencyEnabled = false;
	private static final double kEmergencyPercent = -0.5;
	private static final double kManualShooterPercent = -1.0;
	private static final double kBeltPercent = 0.5;
	private boolean m_assistedActive = false;
	// Siempre encendido (idle): mantiene el shooter girando suave para evitar picos
	// al momento de prenderlo. Sólo aplica cuando el robot está ENABLED.
	private static final double kIdleSpinPercent = -0.15;
	private double m_targetPercent = 0.0;
	private String m_lastShooterCommand = "none";
	private static final String kIdleSpinEnabledKey = "Shooter/IdleSpin/Enabled";
	private static final String kIdleSpinPercentKey = "Shooter/IdleSpin/Percent";
	private boolean m_idleSpinEnabled = true;
	private boolean m_idleSpinDashboardInitialized = false;
	private boolean m_idleSpinPercentDashboardInitialized = false;
	private double m_lastIdleAppliedPercent = 0.0;
	private String m_requestSource = "none";

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

	private void applyHardStop(String reason) {
		m_assistedActive = false;
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_lastRequestedPercentRaw = 0.0;
		m_targetPercent = 0.0;
		m_rampedPercent = 0.0;
		m_commandedVolts = 0.0;
		m_requestSource = "hardStop";
		m_lastShooterCommand = "HARD_STOP: " + reason;
		m_shooterMotor51.setVoltage(0.0);
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
		// Ramp (subida y bajada). Esto reemplaza el "kick" como método principal anti-pico.
		// (Dejamos variables de kick por compatibilidad, pero ya no se usan aquí.)
		double rampedPercent = applyRamp(requestedPercent);
		boolean requestIsOn = Math.abs(rampedPercent) > 1e-6;
		m_lastRequestedPercent = rampedPercent;

		double targetVolts;
		if (m_emergencyEnabled) {
			m_lastShooterCommand = "percent_blocked_by_emergency";
			m_targetPercent = kEmergencyPercent;
			targetVolts = kNominalVoltage * kEmergencyPercent;
			m_commandedVolts = targetVolts;
			m_shooterMotor51.setVoltage(targetVolts);
			return;
		}

		double appliedPercent = rampedPercent;
		m_targetPercent = requestedPercent; // target = lo que pidió arriba; applied = lo que realmente sale (ramped)
		targetVolts = kNominalVoltage * appliedPercent;
		m_commandedVolts = targetVolts;
		m_requestSource = m_assistedActive ? "assisted" : (requestIsOn ? "manual" : "stop");
		m_lastShooterCommand = String.format(
			"voltageCmd(%.2fV) target(%.3f) appliedRamped(%.3f) rampTimeS(%.2f)",
			targetVolts,
			requestedPercent,
			appliedPercent,
			m_rampTimeS);
		m_shooterMotor51.setVoltage(targetVolts);
	}

	/**
	 * Set shooter percent WITHOUT startup-kick logic.
	 *
	 * <p>Usado para el idle spin, para que no se convierta en un "kick" fuerte cada vez que
	 * se prende/apaga desde Shuffleboard.
	 */
	private void setShooterPercentNoKick(double requestedPercent) {
		if (m_hardStop) {
			applyHardStop("blocked setShooterPercentNoKick(" + requestedPercent + ")");
			return;
		}
		// Usar la misma rampa para idle/stop para que no regrese al idle de putazo.
		double rampedPercent = applyRamp(requestedPercent);
		m_lastRequestedPercent = rampedPercent;
		m_startupKickRemaining = 0;

		if (m_emergencyEnabled) {
			m_lastShooterCommand = "percent_blocked_by_emergency";
			m_targetPercent = kEmergencyPercent;
			double targetVolts = kNominalVoltage * kEmergencyPercent;
			m_commandedVolts = targetVolts;
			m_shooterMotor51.setVoltage(targetVolts);
			return;
		}

		m_targetPercent = requestedPercent;
		double targetVolts = kNominalVoltage * rampedPercent;
		m_commandedVolts = targetVolts;
		m_requestSource = (Math.abs(requestedPercent) > 1e-6) ? "idle" : "idle_stop";
		m_lastShooterCommand = String.format(
			"idle_voltageCmd(%.2fV) target(%.3f) appliedRamped(%.3f) rampTimeS(%.2f)",
			targetVolts,
			requestedPercent,
			rampedPercent,
			m_rampTimeS);
		m_shooterMotor51.setVoltage(targetVolts);
	}

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
		m_assistedActive = false;
		m_requestSource = "return_to_idle";
		// IMPORTANTE: NO reseteamos la rampa a 0 aqui.
		// Queremos que el shooter baje SUAVE desde el valor actual hacia el idle (-0.15)
		// respetando Shooter/Ramp/TimeS (tipicamente 0.5s).
		// Usamos el valor de Shuffleboard si existe; si no, el default.
		// OJO: si en Shuffleboard alguien dejo -0.40, se va a quedar ahi. Por eso publicamos
		// el setpoint usado para que puedas confirmarlo.
		double idlePercent = SmartDashboard.getNumber(kIdleSpinPercentKey, kIdleSpinPercent);
		idlePercent = MathUtil.clamp(idlePercent, -0.40, 0.0);
		SmartDashboard.putNumber("Shooter/IdleSpin/ReturnTargetPercent", idlePercent);
		if (m_idleSpinEnabled) {
			// Usa la misma rampa para transicionar al idle.
			setShooterPercentNoKick(idlePercent);
		} else {
			setShooterPercentNoKick(0.0);
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
		m_startupKickRemaining = 0;
		m_lastRequestedPercent = 0.0;
		m_lastRequestedPercentRaw = 0.0;
		m_rampedPercent = 0.0;
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
		if (m_hardStop) {
			applyHardStop("blocked toggleEmergencyShooter");
			return;
		}
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
		m_requestSource = "stop";
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
		// Ramp time tuneable
		if (!m_rampDashboardInitialized) {
			SmartDashboard.putNumber(kRampTimeKey, m_rampTimeS);
			m_rampDashboardInitialized = true;
		}
		m_rampTimeS = SmartDashboard.getNumber(kRampTimeKey, m_rampTimeS);
		m_rampTimeS = MathUtil.clamp(m_rampTimeS, 0.05, 2.0);

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

		// Toggle desde Shuffleboard:
		//  - La primera vez publicamos el default (true)
		//  - Después SOLO leemos, para no pisar lo que el usuario cambie.
		if (!m_idleSpinDashboardInitialized) {
			SmartDashboard.putBoolean(kIdleSpinEnabledKey, m_idleSpinEnabled);
			m_idleSpinDashboardInitialized = true;
		}
		m_idleSpinEnabled = SmartDashboard.getBoolean(kIdleSpinEnabledKey, m_idleSpinEnabled);

		// Percent tuneable desde Shuffleboard.
		// Igual: publicar default una sola vez y luego sólo leer.
		if (!m_idleSpinPercentDashboardInitialized) {
			// Fuerza el default inicial a -0.15 (por si quedo un valor viejo en Shuffleboard).
			SmartDashboard.putNumber(kIdleSpinPercentKey, kIdleSpinPercent);
			m_idleSpinPercentDashboardInitialized = true;
		}
		double idlePercent = SmartDashboard.getNumber(kIdleSpinPercentKey, kIdleSpinPercent);
		// Seguridad: idle no debería ser positivo ni demasiado alto.
		idlePercent = edu.wpi.first.math.MathUtil.clamp(idlePercent, -0.40, 0.0);

		// Si el robot está ENABLED y no hay un comando que esté pidiendo shooter,
		// mantener un giro mínimo constante.
		// Importante: NO correr en disabled (por seguridad) y NO pelear con emergencia.
		boolean robotEnabled = edu.wpi.first.wpilibj.DriverStation.isEnabled();
		boolean requestIsOff = Math.abs(m_lastRequestedPercent) <= 1e-6;
		m_lastIdleAppliedPercent = 0.0;
		if (!m_hardStop && robotEnabled && !m_emergencyEnabled && requestIsOff) {
			if (m_idleSpinEnabled) {
				m_lastIdleAppliedPercent = idlePercent;
				setShooterPercentNoKick(idlePercent);
			} else {
				// Si el idle está deshabilitado y nadie está pidiendo shooter, APAGARLO.
				setShooterPercentNoKick(0.0);
			}
		}

		// En modo voltaje NO necesitamos periodic para controlar, sólo para telemetría.
		SmartDashboard.putBoolean("Shooter/EmergencyEnabled", m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/AssistedEnabled", m_assistedActive && !m_emergencyEnabled);
		SmartDashboard.putBoolean("Shooter/IdleSpinEnabled", m_idleSpinEnabled);
		SmartDashboard.putBoolean("Shooter/HardStopActive", m_hardStop);
		SmartDashboard.putNumber("Shooter/Ramp/TimeS", m_rampTimeS);
		SmartDashboard.putNumber("Shooter/RequestedPercentRaw", m_lastRequestedPercentRaw);
		SmartDashboard.putNumber("Shooter/AppliedPercentRamped", m_rampedPercent);
		SmartDashboard.putNumber("Shooter/IdleSpinPercent", idlePercent);
		SmartDashboard.putNumber("Shooter/IdleSpin/AppliedPercent", m_lastIdleAppliedPercent);
		SmartDashboard.putNumber("Shooter/IdleSpin/AppliedVolts", kNominalVoltage * m_lastIdleAppliedPercent);
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
		SmartDashboard.putString("Shooter/RequestSource", m_requestSource);
	}
}
