package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AsistedShooter extends SubsystemBase {
	private final Camara m_camara;
	private static final double kRpmSlewRatePerSec = 1500.0;
	private final SlewRateLimiter m_rpmLimiter = new SlewRateLimiter(kRpmSlewRatePerSec);
	private static final double kMaxShootDistanceMeters = 4.0;
	private static final double kMinDistanceMeters = 0.30;
	// Exponential mapping params fitted to points:
	//  d=2m -> 0.53
	//  d=3m -> 0.60
	//  d=4m -> 0.69
	// We fit percent(d) = A * exp(B * d) + C
	private static final double kExpA = 0.1482098765;
	private static final double kExpB = 0.2513144283; // ln(9/7)
	private static final double kExpC = 0.285;

	// Smoothing for percent output (units: percent per second)
	private static final double kPercentSlewRatePerSec = 1.0;
	private final SlewRateLimiter m_percentLimiter = new SlewRateLimiter(kPercentSlewRatePerSec);

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
		double dist = getAutoAimDistanceMeters();
		// Clamp distancia válida para la curva
		double d = MathUtil.clamp(dist, kMinDistanceMeters, kMaxShootDistanceMeters);
		// percent magnitude (positivo)
		double rawPercent = (kExpA * Math.exp(kExpB * d)) + kExpC;
		rawPercent = MathUtil.clamp(rawPercent, 0.0, 1.0);
		// Aplicar suavizado (limitar cambio por segundo)
		double smoothed = m_percentLimiter.calculate(rawPercent);
		// Shooter forward convention: NEGATIVO
		return -MathUtil.clamp(smoothed, -1.0, 1.0);
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
			targetPercentRaw = (kExpA * Math.exp(kExpB * dist)) + kExpC;
		}
		SmartDashboard.putNumber("AssistShooter/TargetRPMRaw", targetRpmRaw);
		SmartDashboard.putNumber("AssistShooter/TargetPercentRaw", targetPercentRaw);
	}

}

