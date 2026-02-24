
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Vision-based alignment helpers.
 *
 * Current use: "Trench" alignment using the closest visible AprilTag from the
 * trench tag set.
 */
public class Alignment {
    private final DriveSubsystem m_drive;
    private final Camara m_camara;

    public Alignment(DriveSubsystem drive, Camara camara) {
        this.m_drive = drive;
        this.m_camara = camara;
    }

    /**
     * While running, uses a "Trench" AprilTag (one of: 7, 6, 12, 1, 17, 26, 22, 23)
     * and drives to be aligned at a 1.0m standoff.
     *
     * This is intended for the game task you called "TRENCH": multiple tags can
     * represent
     * the same alignment objective, so we pick the closest visible one.
     * - Rotation: yaw -> 0
     * - Translation (field-relative):
     * * forward/back to reach desired range
     * * left/right to center laterally
     */
    public Command alignToTrenchOneMeter() {
        return Commands.runEnd(
                () -> {
                    // Tunables
                    final double desiredRangeM = SmartDashboard.getNumber("Align/Trench/DesiredRangeM", 1.0);
                    final double kPYaw = SmartDashboard.getNumber("Align/Trench/kPYaw", 0.02);
                    final double kPRange = SmartDashboard.getNumber("Align/Trench/kPRange", 0.9);
                    final double kPStrafe = SmartDashboard.getNumber("Align/Trench/kPStrafe", 1.2);

                    final double maxV = SmartDashboard.getNumber("Align/Trench/MaxV", 0.6);
                    final double maxRot = SmartDashboard.getNumber("Align/Trench/MaxRot", 2.0);

                    final double yawTolDeg = SmartDashboard.getNumber("Align/Trench/YawTolDeg", 2.0);
                    final double rangeTolM = SmartDashboard.getNumber("Align/Trench/RangeTolM", 0.06);
                    final double strafeTolM = SmartDashboard.getNumber("Align/Trench/StrafeTolM", 0.05);

                    // Ensure keys exist (so it's easy to tune from dashboard)
                    SmartDashboard.putNumber("Align/Trench/DesiredRangeM", desiredRangeM);
                    SmartDashboard.putNumber("Align/Trench/kPYaw", kPYaw);
                    SmartDashboard.putNumber("Align/Trench/kPRange", kPRange);
                    SmartDashboard.putNumber("Align/Trench/kPStrafe", kPStrafe);
                    SmartDashboard.putNumber("Align/Trench/MaxV", maxV);
                    SmartDashboard.putNumber("Align/Trench/MaxRot", maxRot);
                    SmartDashboard.putNumber("Align/Trench/YawTolDeg", yawTolDeg);
                    SmartDashboard.putNumber("Align/Trench/RangeTolM", rangeTolM);
                    SmartDashboard.putNumber("Align/Trench/StrafeTolM", strafeTolM);

                    if (!m_camara.hasTrenchTag()) {
                        SmartDashboard.putBoolean("Align/Trench/Active", false);
                        m_drive.drive(0, 0, 0, true);
                        return;
                    }

                    SmartDashboard.putBoolean("Align/Trench/Active", true);

                    // Vision measurements (camera frame): X forward, Y left.
                    // We use the best (closest) Trench tag visible.
                    final int tagId = m_camara.getTrenchTagId();
                    final double yawDeg = m_camara.getTrenchYawDeg();
                    final double xM = m_camara.getTrenchX_M();
                    final double yM = m_camara.getTrenchY_M();

                    // Errors
                    final double rangeErrorM = xM - desiredRangeM;
                    final double strafeErrorM = yM;

                    // Deadbands / tolerances to avoid hunting
                    final double yawErrDeg = (Math.abs(yawDeg) < yawTolDeg) ? 0.0 : yawDeg;
                    final double rangeErrM = (Math.abs(rangeErrorM) < rangeTolM) ? 0.0 : rangeErrorM;
                    final double strafeErrM = (Math.abs(strafeErrorM) < strafeTolM) ? 0.0 : strafeErrorM;

                    // Controllers (simple P)
                    double vx = -kPRange * rangeErrM; // forward (+) to reduce positive range error
                    double vy = -kPStrafe * strafeErrM; // right is negative Y
                    double rot = -kPYaw * yawErrDeg;

                    vx = MathUtil.clamp(vx, -maxV, maxV);
                    vy = MathUtil.clamp(vy, -maxV, maxV);
                    rot = MathUtil.clamp(rot, -maxRot, maxRot);

                    SmartDashboard.putNumber("Align/Trench/YawDeg", yawDeg);
                    SmartDashboard.putNumber("Align/Trench/TagId", tagId);
                    SmartDashboard.putNumber("Align/Trench/RangeErrorM", rangeErrorM);
                    SmartDashboard.putNumber("Align/Trench/StrafeErrorM", strafeErrorM);
                    SmartDashboard.putNumber("Align/Trench/VxCmd", vx);
                    SmartDashboard.putNumber("Align/Trench/VyCmd", vy);
                    SmartDashboard.putNumber("Align/Trench/RotCmd", rot);

                    // Drive field-relative.
                    m_drive.drive(vx, vy, rot, true);
                },
                () -> {
                    SmartDashboard.putBoolean("Align/Trench/Active", false);
                    m_drive.drive(0, 0, 0, true);
                },
                m_drive);
    }
}
