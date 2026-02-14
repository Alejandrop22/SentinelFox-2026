package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Rotates the robot to face the AprilTag based on camera x/y using angle = atan2(x, y).
 * - Only runs if a target is present and distance <= 3.0 m.
 * - Computes desired absolute heading in degrees and rotates the robot there using a PID loop.
 */
public class AimToAprilTag extends Command {
  private final DriveSubsystem m_drive;
  private final PhotonCamera m_camera;
  // PID gains: tuned for moderate correction. Reduced slightly from previous so it
  // doesn't overshoot too much while still correcting effectively.
  private final PIDController m_turnController = new PIDController(0.05, 0.0, 0.003);
  private double m_desiredAngleDeg = 0.0;
  // Scale the PID output so we don't rotate at full speed; helps detection and stability.
  // Allow a higher rotation output so the robot can correct more strongly while moving.
  private static final double kRotationScale = 0.8;
  // When translation magnitude is high, reduce rotation scale by this factor at full speed.
  // Effective scale = kRotationScale * (1 - kTranslationScaleFactor * transMag)
  private static final double kTranslationScaleFactor = 0.7;
  // Minimum rotation scale even when moving fast
  private static final double kMinRotationScale = 0.2;
  // Rate limit for rotation override per execute() (fraction per cycle)
  private static final double kMaxRotationDelta = 0.04;
  // Simple low-pass smoothing factor (0..1) applied to the override (higher=more responsive)
  private static final double kSmoothingAlpha = 0.7;
  // Previous applied override (fraction)
  private double m_prevAppliedOverride = 0.0;
  // For gradient-based minimization of projected distance r = sqrt(x^2 + y^2)
  private double m_rLP = Double.NaN; // low-pass r
  private static final double kRLowpassAlpha = 0.4;
  private static final double kREps = 0.002; // meters change threshold considered significant
  private double m_prevRLP = Double.NaN;
  private int m_stableCounter = 0;
  private final int kStableCountNeeded = 5; // cycles
  private final double kAngleToleranceDeg = 2.0;
  // Hysteresis deadband to avoid small jitter when near target
  private static final double kAngleHoldDeadbandDeg = 0.8;
  private static final double kAngleResumeDeg = 1.6;
  private boolean m_holdActive = false;
  private boolean m_abort = false;
  private final boolean m_continuous;

  public AimToAprilTag(DriveSubsystem drive, String cameraName) {
    this(drive, cameraName, false);
  }

  public AimToAprilTag(DriveSubsystem drive, String cameraName, boolean continuous) {
    m_drive = drive;
    m_camera = new PhotonCamera(cameraName);
    // NOTE: we intentionally do NOT take DriveSubsystem as a requirement here.
    // The goal is to allow the driver's translation inputs to continue while
    // this command only overrides rotation via DriveSubsystem.setRotationOverride().
    m_turnController.enableContinuousInput(-180.0, 180.0);
    m_continuous = continuous;
  }

  @Override
  public void initialize() {
    // Check for a valid target and distance now; if not valid we will cancel in initialize
    m_abort = false;
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (!result.hasTargets()) {
      // Don't abort if we don't see the target at initialization. Keep the command
      // running so it can re-acquire the tag later. We set the desired angle to the
      // current heading so we don't immediately rotate.
      DriverStation.reportWarning("AimToAprilTag: No Target (will keep running)", false);
      SmartDashboard.putString("Teleop/AimInfo", "No Target (waiting)");
      double currentHeadingNoTarget = m_drive.getHeading();
      m_desiredAngleDeg = normalizeDeg(currentHeadingNoTarget);
      m_turnController.reset();
      m_stableCounter = 0;
      return;
    }

    PhotonTrackedTarget target = result.getBestTarget();
    var t = target.getBestCameraToTarget();
    double x = t.getX();
    double y = t.getY();
    double z = t.getZ();
    int tagId = target.getFiducialId();
  double distancia = Math.sqrt(x * x + y * y + z * z); // kept for telemetry

    // Previously we aborted if the target was >3.0m away. User requested
    // removing any distance limit so we always attempt to correct.

  // Compute desired relative angle from camera-to-target x,y.
  // Use atan2(y, x) so the angle is measured as atan2(left, forward) which gives
  // positive = left, negative = right relative to camera forward.
  // atan2 returns radians; convert to degrees. This angle is relative to camera forward (0 deg).
  double angleRad = Math.atan2(y, x);
  double angleDeg = Math.toDegrees(angleRad);
    // Convert to an absolute desired robot heading by adding the current heading so that rotating
    // the robot to this absolute heading will center the tag in front.
    double currentHeading = m_drive.getHeading();
    m_desiredAngleDeg = normalizeDeg(currentHeading + angleDeg);
  // reset controller state; we'll use the controller with (measurement=current, setpoint=desired)
  m_turnController.reset();
  m_stableCounter = 0;

    // Detailed math message: include detected x,y,z, distance, atan2 (rad/deg), and final target angle
    String info = String.format(
        "Aim id=%d / x=%.3fm y=%.3fm z=%.3fm / dist=%.2fm / atan2(x,y)=%.4frad(%.2fdeg) -> target=%.2fdeg",
        tagId, x, y, z, distancia, angleRad, angleDeg, m_desiredAngleDeg);
    SmartDashboard.putString("Teleop/AimInfo", info);
    DriverStation.reportWarning(info, false);

  // Also publish a concise "Yendo a (DIRECTO): <angle>  X:<x> Y:<y>" message like you requested
  String directMsg = String.format("Yendo a (DIRECTO): %.2f  X: %.3fm  Y: %.3fm", m_desiredAngleDeg, x, y);
  SmartDashboard.putString("Teleop/AimDirect", directMsg);
  DriverStation.reportWarning(directMsg, false);
  }

  @Override
  public void execute() {
  // If running in continuous mode, recompute desired heading each cycle from the latest camera pose
    if (m_continuous) {
      PhotonPipelineResult result = m_camera.getLatestResult();
      if (!result.hasTargets()) {
        // Lost target temporarily: stop rotating (clear override) but do NOT abort.
        DriverStation.reportWarning("AimToAprilTag: Lost Target", false);
        m_drive.clearRotationOverride();
        return;
      }
      PhotonTrackedTarget target = result.getBestTarget();
      // If we had been searching and now reacquired, stop searching state
      // If we reacquired the target, continue normal correction.
      var t = target.getBestCameraToTarget();
      double x = t.getX();
      double y = t.getY();
      double z = t.getZ();
  double distancia = Math.sqrt(x * x + y * y + z * z); // value kept for telemetry
  SmartDashboard.putNumber("Teleop/AimDistance", distancia);
      // No distance cutoff: always attempt to correct regardless of distance.
      double angleRad = Math.atan2(y, x);
      double angleDeg = Math.toDegrees(angleRad);
      double currentHeading = m_drive.getHeading();
      m_desiredAngleDeg = normalizeDeg(currentHeading + angleDeg);
      // Update low-pass of projected distance r = sqrt(x^2 + y^2)
      double r = Math.hypot(x, y);
      if (Double.isNaN(m_rLP)) {
        m_rLP = r;
      } else {
        // store previous for delta computation
        m_prevRLP = m_rLP;
        m_rLP = kRLowpassAlpha * r + (1.0 - kRLowpassAlpha) * m_rLP;
      }
    }
    double current = m_drive.getHeading(); // degrees
    // Use PID properly: measurement = current heading, setpoint = desired absolute heading.
    double pidOut = m_turnController.calculate(current, m_desiredAngleDeg);
  // clamp to -1..1 (drive expects fraction of max angular speed)
  double output = Math.max(-1.0, Math.min(1.0, pidOut));
    // Adapt rotation scale based on current operator translation magnitude (0..1)
  double transMag = m_drive.getLastTranslationMagnitude();
  double dynamicScale = kRotationScale * (1.0 - kTranslationScaleFactor * transMag);
  if (dynamicScale < kMinRotationScale) dynamicScale = kMinRotationScale;
    // Gradient-based adjustment: if r is increasing we may be rotating the wrong way.
    double targetOverride = output * dynamicScale;
    if (!Double.isNaN(m_rLP) && !Double.isNaN(m_prevRLP)) {
      double rDelta = m_rLP - m_prevRLP;
      // determine base sign from PID output or previous applied override
      int baseSign = (output > 0.0) ? 1 : (output < 0.0 ? -1 : (m_prevAppliedOverride >= 0.0 ? 1 : -1));
      double mag = Math.abs(output);
      if (rDelta > kREps) {
        // r increased -> we're going the wrong way; invert and attenuate
        baseSign = -baseSign;
        mag *= 0.6;
      } else if (rDelta < -kREps) {
        // r decreased -> keep direction and slightly amplify (cap at 1)
        mag = Math.min(1.0, mag * 1.05);
      }
      targetOverride = baseSign * mag * dynamicScale;
    }

  // Limit sudden changes in rotation to keep motion smooth and avoid destabilizing swerve modules.
  double maxDelta = kMaxRotationDelta; // per execute() call
  double limited = Math.max(m_prevAppliedOverride - maxDelta, Math.min(m_prevAppliedOverride + maxDelta, targetOverride));

  // Low-pass filter to smooth the applied rotation further
  double applied = (1.0 - kSmoothingAlpha) * m_prevAppliedOverride + kSmoothingAlpha * limited;
  m_prevAppliedOverride = applied;

  // Hysteresis: if error is very small, hold rotation at zero to avoid jitter.
  double rawError = m_desiredAngleDeg - current;
  double error = normalizeDeg(rawError);
  if (Math.abs(error) < kAngleHoldDeadbandDeg) {
    // hold — clear small corrections
    m_prevAppliedOverride = 0.0;
    m_drive.setRotationOverride(0.0);
  } else if (Math.abs(error) < kAngleResumeDeg && m_prevAppliedOverride == 0.0) {
    // still in hold window; don't apply tiny corrections
    m_drive.setRotationOverride(0.0);
  } else {
    m_drive.setRotationOverride(applied);
  }

    // publish debug: desired/current/error
    String dbg = String.format("AimDbg: desired=%.2f current=%.2f delta=%.2f", m_desiredAngleDeg, current, error);
    SmartDashboard.putString("Teleop/AimDbg", dbg);
    DriverStation.reportWarning(dbg, false);

    if (Math.abs(error) < kAngleToleranceDeg) {
      m_stableCounter++;
    } else {
      m_stableCounter = 0;
    }
  }

  @Override
  public boolean isFinished() {
    // If running in continuous mode, don't finish just because we're "stable" — keep
    // correcting in real time until the command is explicitly canceled or aborted.
    if (m_continuous) {
      return m_abort; // only finish on explicit abort (e.g., serious error)
    }
    return m_abort || (m_stableCounter >= kStableCountNeeded);
  }

  @Override
  public void end(boolean interrupted) {
    // Clear any rotation override so the driver regains control.
    m_drive.clearRotationOverride();
    // Also stop motion explicitly as a fallback.
    m_drive.drive(0.0, 0.0, 0.0, true);
    // Reset smoothing state
    m_prevAppliedOverride = 0.0;
    String endMsg = interrupted ? "Aim Interrupted" : "Aim Completed";
    SmartDashboard.putString("Teleop/AimInfo", endMsg);
    DriverStation.reportWarning(endMsg, false);
  }

  private double normalizeDeg(double a) {
    double ang = a;
    while (ang > 180.0) ang -= 360.0;
    while (ang <= -180.0) ang += 360.0;
    return ang;
  }
}
