package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AsistedShooter extends SubsystemBase {
	private final Camara m_camara;

	// Smooth percent output so vision noise doesn't cause jumps.
	// Units: (percent output) per second. Keep this fairly high so it doesn't feel sluggish.
	private static final double kPercentSlewRatePerSec = 2.5;
	private final SlewRateLimiter m_percentLimiter = new SlewRateLimiter(kPercentSlewRatePerSec);

	// Calibration points (distance meters -> percent output)
	private static final double kD1 = 1.0;
	private static final double kV1 = -0.550;
	private static final double kD2 = 2.0;
	private static final double kV2 = -0.625;
	private static final double kD3 = 3.0;
	private static final double kV3 = -0.700;

	// Max allowed distance to shoot
	private static final double kMaxShootDistanceMeters = 4.0;

	// Minimum distance clamp to avoid weird extrapolation very near tag.
	private static final double kMinDistanceMeters = 0.5;

	public AsistedShooter(Camara camara) {
		m_camara = camara;
	}

	/**
	 * Returns true if we are allowed to spin up for shooting.
	 * Requirement: if >4m, don't let it even turn on.
	 */
	public boolean canShootNow() {
		if (!m_camara.hasTag1()) {
			// If Tag1 is gone, reset smoothing so we don't "carry" old output.
			m_percentLimiter.reset(0.0);
			return false;
		}
		double dist = getTag1DistanceMeters();
		if (!(dist > 0.0 && dist <= kMaxShootDistanceMeters)) {
			m_percentLimiter.reset(0.0);
			return false;
		}
		return true;
	}

	/**
	 * Get distance estimate for Tag 1.
	 * Uses Camara's computed distance to target (implementation is whatever Camara provides).
	 */
	public double getTag1DistanceMeters() {
		// Camara currently doesn't store per-tag distance; it stores best-target distance.
		// We'll use that as your distance estimate while Tag1 is present.
		return m_camara.getDistanceToTarget();
	}

	/** Map distance (m) -> shooter percent output. */
	public double percentForDistance(double distanceMeters) {
		// Caller must ensure distance is valid and within range. We still clamp to keep math sane.
		double d = MathUtil.clamp(distanceMeters, kMinDistanceMeters, kMaxShootDistanceMeters);

		// Piecewise linear interpolation through the three calibration points.
		if (d <= kD2) {
			return lerp(kD1, kV1, kD2, kV2, d);
		}
		if (d <= kD3) {
			return lerp(kD2, kV2, kD3, kV3, d);
		}

		// For 3m..4m extend with the same slope as (2..3). This matches your "saca la funcion" request.
		double slope23 = (kV3 - kV2) / (kD3 - kD2);
		double vAt4 = kV3 + slope23 * (kMaxShootDistanceMeters - kD3);
		return lerp(kD3, kV3, kMaxShootDistanceMeters, vAt4, d);
	}

	private static double lerp(double x1, double y1, double x2, double y2, double x) {
		if (x2 == x1) {
			return y1;
		}
		double t = (x - x1) / (x2 - x1);
		return y1 + t * (y2 - y1);
	}

	/** Stop the assisted shooter motor. */
	public void stop() {
		// Motor control lives in Shooter subsystem now.
		m_percentLimiter.reset(0.0);
	}

	/**
	 * Convenience: compute the percent output we'd like to apply right now.
	 * RobotContainer decides whether to apply it (assisted allowed) and which motor controls it.
	 */
	public double getDesiredPercent() {
		double dist = getTag1DistanceMeters();
		double raw = percentForDistance(dist);
		return m_percentLimiter.calculate(raw);
	}

	/**
	 * Run assisted shooter based on Tag1 distance.
	 * Caller (RobotContainer) decides whether assisted mode is allowed.
	 */
	public void runAssisted() {
		double dist = getTag1DistanceMeters();
		double rawPercent = percentForDistance(dist);
		double percent = m_percentLimiter.calculate(rawPercent);

		SmartDashboard.putBoolean("AssistShooter/HasTag1", m_camara.hasTag1());
		SmartDashboard.putNumber("AssistShooter/DistanceM", dist);
		SmartDashboard.putNumber("AssistShooter/RawPercent", rawPercent);
		SmartDashboard.putNumber("AssistShooter/Percent", percent);
		SmartDashboard.putString("AssistShooter/State", "ASSISTED");
	}

	@Override
	public void periodic() {
		// No-op. Assisted firing is commanded by RobotContainer while RT is held.
	}

}

