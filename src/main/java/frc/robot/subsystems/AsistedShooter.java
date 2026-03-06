package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AsistedShooter extends SubsystemBase {
	private final Camara m_camara;
	private static final double kRpmSlewRatePerSec = 1500.0;
	private final SlewRateLimiter m_rpmLimiter = new SlewRateLimiter(kRpmSlewRatePerSec);
	private static final double kMaxShootDistanceMeters = 5.0;
	private static final double kMinDistanceMeters = 0.30;
	// Curva nueva (100%):
	// percentMag(x) = 0.01x^2 + 0.02x + 0.45
	// donde x está en METROS.
	private static final double kPercentA = 0.01;
	private static final double kPercentB = 0.02;
	private static final double kPercentC = 0.45;

	// Smoothing for percent output (units: percent per second)
	// Mas bajo = mas estable (menos "fluctuacion" audible) pero responde mas lento.
	private static final double kPercentSlewRatePerSec = 0.35;
	private final SlewRateLimiter m_percentLimiter = new SlewRateLimiter(kPercentSlewRatePerSec);

	// Si el cambio es muy pequeno, no lo persigas (reduce ruido por mediciones de distancia inestables)
	private static final double kPercentChangeDeadband = 0.02;
	private double m_lastPercentCmd = 0.0;

	// Multiplicador tuneable (Shuffleboard): permite ajustar el resultado final sin tocar la curva.
	// Ejemplo: 0.95 = 5% menos velocidad.
	private static final String kAssistPercentMultiplierKey = "AssistShooter/PercentMultiplier";
	private boolean m_multiplierDashboardInitialized = false;

	//multiplier del SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTER
	private double m_percentMultiplier = 1.9;

	private void ensureMultiplierRead() {
		if (!m_multiplierDashboardInitialized) {
			SmartDashboard.putNumber(kAssistPercentMultiplierKey, m_percentMultiplier);
			m_multiplierDashboardInitialized = true;
		}
		m_percentMultiplier = SmartDashboard.getNumber(kAssistPercentMultiplierKey, m_percentMultiplier);
		m_percentMultiplier = MathUtil.clamp(m_percentMultiplier, 0.70, 1.30);
		SmartDashboard.putNumber("AssistShooter/PercentMultiplierApplied", m_percentMultiplier);
	}

	/** Aplica el multiplicador de Shuffleboard a un percent (negativo = forward). */
	public double applyPercentMultiplier(double percentRaw) {
		ensureMultiplierRead();
		double scaled = percentRaw * m_percentMultiplier;
		// Shooter forward convention: NEGATIVO. No permitimos positivo aqui.
		scaled = MathUtil.clamp(scaled, -1.0, 0.0);
		SmartDashboard.putNumber("AssistShooter/FallbackPercentRaw", percentRaw);
		SmartDashboard.putNumber("AssistShooter/FallbackPercentScaled", scaled);
		return scaled;
	}

	// Curva confirmada por ti: rpm(x) = -33x^2 + 642x + 3800
	// x está en METROS.
	private static final double kA = -33.0;
	private static final double kB = 642.0;
	private static final double kC = 3800.0;

	// Seguridad de setpoint: evita pedir RPM imposibles o negativas.
	private static final double kMinRpm = 2500.0;
	private static final double kMaxRpm = 6500.0;

	public AsistedShooter(Camara camara) {
		m_camara = camara;
	}

	public boolean canShootNow() {
		// Usar la distancia del tag seleccionado del AutoAim (26/25/..)
		if (!m_camara.hasAutoAimTag()) {
			m_rpmLimiter.reset(0.0);
			return false;
		}
		double dist = getAutoAimDistanceMeters();
		if (!(dist > 0.0 && dist <= kMaxShootDistanceMeters)) {
			m_rpmLimiter.reset(0.0);
			return false;
		}
		return true;
	}

	public double getAutoAimDistanceMeters() {
		return m_camara.getAutoAimDistanceM();
	}

	public double rpmForDistance(double distanceMeters) {
		double d = MathUtil.clamp(distanceMeters, kMinDistanceMeters, kMaxShootDistanceMeters);

		// rpm(x) = ax^2 + bx + c
		double rpmMag = (kA * d * d) + (kB * d) + kC;
		rpmMag = MathUtil.clamp(rpmMag, kMinRpm, kMaxRpm);
		return -rpmMag; // En este robot, shooter "forward" es NEGATIVO
	}

	public void stop() {
		m_rpmLimiter.reset(0.0);
		m_percentLimiter.reset(0.0);
		m_lastPercentCmd = 0.0;
	}

	public double getDesiredRpm() {
		double dist = getAutoAimDistanceMeters();
		double raw = rpmForDistance(dist);
		return m_rpmLimiter.calculate(raw);
	}

	/**
	 * Salida en porcentaje ([-1..1]) calculada desde la distancia usando una curva
	 * exponencial ajustada a los puntos pedidos por el usuario.
	 *
	 * <p>La función devuelve signo NEGATIVO para preservar la convención del robot
	 * (shooter "forward" es negativo). Se aplica un límite y un suavizado leve.
	 */
	public double getDesiredPercent() {
		ensureMultiplierRead();

		double dist = getAutoAimDistanceMeters();
		// Clamp distancia válida para la curva
		double d = MathUtil.clamp(dist, kMinDistanceMeters, kMaxShootDistanceMeters);
		// percent magnitude (positivo)
		double rawPercent = (kPercentA * d * d) + (kPercentB * d) + kPercentC;
		rawPercent = MathUtil.clamp(rawPercent, 0.0, 1.0);
		// Aplicar suavizado (limitar cambio por segundo)
		double smoothed = m_percentLimiter.calculate(rawPercent);
		double percentCmdRaw = -MathUtil.clamp(smoothed, 0.0, 1.0);
		SmartDashboard.putNumber("AssistShooter/DesiredPercentRaw", percentCmdRaw);

		// Aplicar multiplicador
		double percentCmd = MathUtil.clamp(percentCmdRaw * m_percentMultiplier, -1.0, 0.0);
		SmartDashboard.putNumber("AssistShooter/DesiredPercentScaled", percentCmd);

		// Deadband de cambio: si cambia muy poquito, manten el valor previo.
		if (Math.abs(percentCmd - m_lastPercentCmd) < kPercentChangeDeadband) {
			return m_lastPercentCmd;
		}
		m_lastPercentCmd = percentCmd;
		return percentCmd;
	}

	@Override
	public void periodic() {
		boolean hasTag = m_camara.hasAutoAimTag();
		SmartDashboard.putBoolean("AssistShooter/HasTag", hasTag);

		double dist = hasTag ? getAutoAimDistanceMeters() : 0.0;
		SmartDashboard.putNumber("AssistShooter/DistanceM", dist);

		double targetRpmRaw = 0.0;
		double targetPercentRaw = 0.0;
		if (hasTag) {
			targetRpmRaw = rpmForDistance(dist);
			double d = MathUtil.clamp(dist, kMinDistanceMeters, kMaxShootDistanceMeters);
			targetPercentRaw = (kPercentA * d * d) + (kPercentB * d) + kPercentC;
		}
		SmartDashboard.putNumber("AssistShooter/TargetRPMRaw", targetRpmRaw);
		SmartDashboard.putNumber("AssistShooter/TargetPercentRaw", targetPercentRaw);
	}

}

