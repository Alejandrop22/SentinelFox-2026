package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
// Removed local SparkMax usage; use Shooter subsystem's API instead
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Autonomous command: drive toward AprilTag ID=1 while controlling shooter power based on distance.
 * Behavior:
 *  - Runs continuously until canceled (no internal timeout).
 *  - If no target: drive forward at max speed, shooter OFF.
 *  - If target detected:
 *      - distance < 1.0 m: robot STOP, shooter OFF
 *      - 1.0 <= distance <= 3.0 m: drive speed interpolated from 0@1m to kMaxSpeed@3m;
 *          shooter percent interpolated from 40%@1m to 70%@3m
 *      - distance > 3.0 m: drive at kMaxSpeed, shooter OFF
 */
public class AutoDriveToAprilTag extends Command {
  private final DriveSubsystem m_drive;
  private final PhotonCamera m_camera;
  private final Timer m_timer = new Timer();

  // speed mapping parameters
  private final double kMaxSpeed = 0.5; // 50% of drive max
  private final double kMinDistance = 1.0; // 1m
  private final double kMaxDistance = 3.0; // 3m
  private double m_smoothedSpeed = 0.0; // for smooth accel/decel
  private final double kSmoothingAlpha = 0.08; // smoothing factor (0..1)
  private final PIDController m_distancePid = new PIDController(0.1, 0.0, 0.1);

  // Track last reported info to avoid spamming the Driver Station console
  private String m_lastReportedInfo = "";

  private final Shooter m_shooter;
  // no local SparkMax here; use m_shooter.setLauncherPercent(...) to control hardware

  public AutoDriveToAprilTag(DriveSubsystem drive, Shooter shooter, String cameraName) {
    m_drive = drive;
    m_shooter = shooter;
    m_camera = new PhotonCamera(cameraName);
    addRequirements(m_drive, m_shooter);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    try {
      PhotonPipelineResult result = m_camera.getLatestResult();
      boolean hasTarget = result.hasTargets();

      double desiredSpeed = kMaxSpeed;
      double shooterPercent = 0.0;

      if (hasTarget) {
      PhotonTrackedTarget target = result.getBestTarget();
      int tagId = target.getFiducialId();
      var t = target.getBestCameraToTarget();
      double x = t.getX();
      double y = t.getY();
      double z = t.getZ();
      double distancia = Math.sqrt(x * x + y * y + z * z);

      // Publish a single concise line: "<tagId> / <distancia> m / <shooterPercent>%"
      String info = String.format("%d / %.2f m / %d%%", tagId, distancia, (int) Math.round(shooterPercent * 100.0));
      SmartDashboard.putString("AutoDrive/Info", info);
      // Report to Driver Station only when the message changes (avoid flooding console)
      if (!info.equals(m_lastReportedInfo)) {
        DriverStation.reportWarning("AutoDrive: " + info, false);
        m_lastReportedInfo = info;
      }

      // Determine desired speed & shooter percent based on distance
      if (distancia < kMinDistance) {
        // closer than 1m -> stop robot, shooter OFF
        desiredSpeed = 0.0;
        shooterPercent = 0.0;
        // reset PID to avoid windup
        m_distancePid.reset();
      } else {
        // Use PID to compute speed toward setpoint = kMinDistance (1.0 m)
        double pidOutput = m_distancePid.calculate(distancia, kMinDistance);
        // PID may be negative (if closer than setpoint) — clamp to 0..kMaxSpeed (no reverse)
        desiredSpeed = Math.max(0.0, Math.min(kMaxSpeed, pidOutput));

        if (distancia <= kMaxDistance) {
          // between 1 and 3m -> shooter interpolated
          double frac = (distancia - kMinDistance) / (kMaxDistance - kMinDistance);
          shooterPercent = 0.40 + frac * (0.70 - 0.40);
        } else {
          // farther than 3m -> shooter OFF
          shooterPercent = 0.0;
        }
      }
    } else {
      // No target: continue driving forward at max speed, shooter off
      // No target: publish "No Target / <shooterPercent>%"
      String infoNo = String.format("No Target / %d%%", (int) Math.round(shooterPercent * 100.0));
      SmartDashboard.putString("AutoDrive/Info", infoNo);
      if (!infoNo.equals(m_lastReportedInfo)) {
        DriverStation.reportWarning("AutoDrive: " + infoNo, false);
        m_lastReportedInfo = infoNo;
      }
      desiredSpeed = kMaxSpeed;
      shooterPercent = 0.0;
      // reset PID when we lose target
      m_distancePid.reset();
    }

      // Smooth the speed
      m_smoothedSpeed = (kSmoothingAlpha * desiredSpeed) + ((1 - kSmoothingAlpha) * m_smoothedSpeed);

      // Apply drive command
      m_drive.drive(m_smoothedSpeed, 0.0, 0.0, true);

  // Apply shooter percent (negative for motor direction). shooterPercent==0 => off
  // Shooter subsystem provides the hardware access to avoid duplicate device creation.
  m_shooter.setLauncherPercent(-shooterPercent);
  // Also publish shooter percent as numeric if you want a separate value (optional)
  SmartDashboard.putNumber("AutoDrive/ShooterPercent", shooterPercent);
    } catch (Throwable ex) {
      // Catch any exception to avoid crashing the robot program and report it
      System.err.println("AutoDriveToAprilTag ERROR: " + ex.getMessage());
      ex.printStackTrace();
      SmartDashboard.putString("AutoDrive/Error", ex.toString());
      // Also report exception once to Driver Station
      if (!ex.toString().equals(m_lastReportedInfo)) {
        DriverStation.reportError("AutoDrive ERROR: " + ex.toString(), false);
        m_lastReportedInfo = ex.toString();
      }
      // stop robot for safety
      try {
        m_drive.drive(0.0, 0.0, 0.0, true);
        m_shooter.stopLauncher();
      } catch (Throwable t) {
        // ignore
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false; // run until explicitly cancelled
  }

  @Override
  public void end(boolean interrupted) {
    // stop motors
    m_drive.drive(0.0, 0.0, 0.0, true);
    m_shooter.stopLauncher();
    m_timer.stop();
    // keep only the concise info key; clear it when ending (and report once to DS)
    String endMsg = interrupted ? "Interrupted" : "Ended";
    SmartDashboard.putString("AutoDrive/Info", endMsg);
    if (!endMsg.equals(m_lastReportedInfo)) {
      DriverStation.reportWarning("AutoDrive: " + endMsg, false);
      m_lastReportedInfo = endMsg;
    }
  }
}
