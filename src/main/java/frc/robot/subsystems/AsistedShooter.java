package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AsistedShooter extends SubsystemBase {
	private final Camara m_camara;
	private static final double kPercentSlewRatePerSec = 2.5;
	private final SlewRateLimiter m_percentLimiter = new SlewRateLimiter(kPercentSlewRatePerSec);
	private static final double kD1 = 1.0;
	private static final double kV1 = -0.550;
	private static final double kD2 = 2.0;
	private static final double kV2 = -0.625;
	private static final double kD3 = 3.0;
	private static final double kV3 = -0.700;
	private static final double kMaxShootDistanceMeters = 3.0;
	private static final double kMinDistanceMeters = 0.3;

	public AsistedShooter(Camara camara) {
		m_camara = camara;
	}

	public boolean canShootNow() {
		if (!m_camara.hasTag1()) {
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

	public double getTag1DistanceMeters() {
		return m_camara.getDistanceToTarget();
	}

	public double percentForDistance(double distanceMeters) {
		double d = MathUtil.clamp(distanceMeters, kMinDistanceMeters, kMaxShootDistanceMeters);
		if (d <= kD2) {
			return lerp(kD1, kV1, kD2, kV2, d);
		}
		if (d <= kD3) {
			return lerp(kD2, kV2, kD3, kV3, d);
		}
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

	public void stop() {
		m_percentLimiter.reset(0.0);
	}
	public double getDesiredPercent() {
		double dist = getTag1DistanceMeters();
		double raw = percentForDistance(dist);
		return m_percentLimiter.calculate(raw);
	}
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
	}

}

