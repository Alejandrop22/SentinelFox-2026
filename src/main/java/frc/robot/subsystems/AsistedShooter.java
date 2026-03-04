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
	 * Salida en porcentaje ([-1..1]) equivalente al RPM deseado.
	 *
	 * <p>Como ya no estamos usando control por velocidad, convertimos el RPM deseado a un
	 * porcentaje aproximado usando un máximo nominal.
	 */
	public double getDesiredPercent() {
		final double kNominalMaxRpmMagnitude = 6000.0;
		return MathUtil.clamp(getDesiredRpm() / kNominalMaxRpmMagnitude, -1.0, 1.0);
	}

	@Override
	public void periodic() {
		boolean hasTag = m_camara.hasAutoAimTag();
		SmartDashboard.putBoolean("AssistShooter/HasTag", hasTag);

		double dist = hasTag ? getAutoAimDistanceMeters() : 0.0;
		SmartDashboard.putNumber("AssistShooter/DistanceM", dist);

		double targetRpmRaw = 0.0;
		if (hasTag) {
			targetRpmRaw = rpmForDistance(dist);
		}
		SmartDashboard.putNumber("AssistShooter/TargetRPMRaw", targetRpmRaw);
	}

}

