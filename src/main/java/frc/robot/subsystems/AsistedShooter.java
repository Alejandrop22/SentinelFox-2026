package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AsistedShooter extends SubsystemBase {
	private final Camara m_camara;
	private static final double kPercentSlewRatePerSec = 2.5;
	private final SlewRateLimiter m_percentLimiter = new SlewRateLimiter(kPercentSlewRatePerSec);
	private static final double kMaxShootDistanceMeters = 4.0;
	private static final double kMinDistanceMeters = 0.3;

	// Curva basada en TUS datos medidos (interpolación lineal por tramos).
	// Distancias:
	//  - d1 = 23.5 in  -> shooter 0.56
	//  - d2 = 1.0m + 23.5 in -> shooter 0.65
	//  - d3 = 2.0m + 23.5 in -> shooter 0.73
	// NOTA: "altura máxima mínima 1.90m" no se puede garantizar solo con la velocidad del shooter,
	// porque depende del ángulo (Angular) y de la física real. Pero sí podemos corregir la curva para
	// que NO se pase de fuerza respecto a las pruebas.
	private static final double kInchesToMeters = 0.0254;
	private static final double kD1_M = 23.5 * kInchesToMeters;
	private static final double kD2_M = 1.0 + (23.5 * kInchesToMeters);
	private static final double kD3_M = 2.0 + (23.5 * kInchesToMeters);

	private static final double kP1 = 0.56;
	private static final double kP2 = 0.65;
	private static final double kP3 = 0.73;

	// Seguridad: no dejar que el assisted pase de lo mas alto que mediste (evita que quede muy fuerte).
	private static final double kMinPercentMag = 0.45;
	private static final double kMaxPercentMag = kP3;

	// Ajuste global (multiplica toda la curva). Si "en general va mas rapido", baja este numero.
	// Lo publicamos a SmartDashboard para poder tunear en la cancha.
	private static final String kShootScaleKey = "AssistShooter/ShootScale";
	private static final double kDefaultShootScale = 0.85;

	public AsistedShooter(Camara camara) {
		m_camara = camara;
	}

	public boolean canShootNow() {
		// Usar la distancia del tag seleccionado del AutoAim (26/25/..)
		if (!m_camara.hasAutoAimTag()) {
			m_percentLimiter.reset(0.0);
			return false;
		}
		double dist = getAutoAimDistanceMeters();
		if (!(dist > 0.0 && dist <= kMaxShootDistanceMeters)) {
			m_percentLimiter.reset(0.0);
			return false;
		}
		return true;
	}

	public double getAutoAimDistanceMeters() {
		return m_camara.getAutoAimDistanceM();
	}

	public double percentForDistance(double distanceMeters) {
		double d = MathUtil.clamp(distanceMeters, kMinDistanceMeters, kMaxShootDistanceMeters);

		double percentMag;
		if (d <= kD1_M) {
			// Si estas mas cerca que tu primer punto medido, BAJAR un poco la velocidad.
			// Interpolamos desde kMinPercentMag (muy cerca) hasta kP1 en d1.
			percentMag = lerp(kMinDistanceMeters, kMinPercentMag, kD1_M, kP1, d);
		} else if (d <= kD2_M) {
			percentMag = lerp(kD1_M, kP1, kD2_M, kP2, d);
		} else if (d <= kD3_M) {
			percentMag = lerp(kD2_M, kP2, kD3_M, kP3, d);
		} else {
			// Más allá de tu último punto medido, mantener constante para no sobre-potenciar.
			percentMag = kP3;
		}

		// Escala global (tuning)
		double shootScale = SmartDashboard.getNumber(kShootScaleKey, kDefaultShootScale);
		percentMag *= MathUtil.clamp(shootScale, 0.2, 1.2);

		percentMag = MathUtil.clamp(percentMag, kMinPercentMag, kMaxPercentMag);
		return -percentMag; // shooter "forward" es negativo en este robot
	}

	private static double lerp(double x1, double y1, double x2, double y2, double x) {
		if (x2 == x1) {
			return y1;
		}
		double t = (x - x1) / (x2 - x1);
		return y1 + (t * (y2 - y1));
	}

	public void stop() {
		m_percentLimiter.reset(0.0);
	}

	public double getDesiredPercent() {
		double dist = getAutoAimDistanceMeters();
		double raw = percentForDistance(dist);
		return m_percentLimiter.calculate(raw);
	}

	@Override
	public void periodic() {
	}

}

