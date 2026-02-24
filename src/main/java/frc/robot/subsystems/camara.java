package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camara extends SubsystemBase {

    private final PhotonCamera m_camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    // =========================
    // AutoAim tags: tags válidos para apuntar (elegimos el más cercano en cada frame).
    // =========================
    private static final int[] kAutoAimTagIds = new int[] {8, 5, 4, 3, 2, 11, 9, 10, 18, 19, 20, 21, 24, 25, 26, 27};

    private boolean has_autoaim_tag;
    private int autoaim_tag_id;
    private double autoaim_yaw_deg;
    private double autoaim_distance_m;

    // =========================
    // "Trench" tags: IDs que se usan para alinear el robot en esa zona.
    // =========================
    // Nota: lo dejamos aquí para que el lector entienda que estos tags existen con un propósito de juego.
    private static final int[] kTrenchTagIds = new int[] {7, 6, 12, 1, 17, 26, 22, 23};

    private boolean has_trench_tag;
    private int trench_tag_id;
    private double trench_yaw_deg;
    private double trench_distance_m;
    private double trench_x_m;
    private double trench_y_m;

    /**
     * Offset horizontal de montaje de la cámara (grados) para compensar que no está centrada.
     * Si la cámara está físicamente a la derecha del centro del robot, normalmente necesitarás un offset NEGATIVO
     * (apunta un poco más a la izquierda) pero depende de tu montaje y del eje de yaw; ajusta en la práctica.
     */
    private static final double kCameraYawOffsetDeg = 5.0;

    /** True si se ve el tag elegido para AutoAim (cualquiera de kAutoAimTagIds). */
    public boolean hasAutoAimTag() {
        return has_autoaim_tag;
    }

    /** ID del tag de AutoAim elegido (el más cercano visible). 0 si no hay. */
    public int getAutoAimTagId() {
        return autoaim_tag_id;
    }

    /** Yaw (grados) del tag elegido. */
    public double getAutoAimYawDeg() {
        return autoaim_yaw_deg;
    }

    /** Distancia (m) al tag elegido. */
    public double getAutoAimDistanceM() {
        return autoaim_distance_m;
    }

    // =========================
    // Trench (multi-tag) getters
    // =========================

    /** True si hay algún tag válido de Trench visible en el frame actual. */
    public boolean hasTrenchTag() {
        return has_trench_tag;
    }

    /** ID del tag de Trench elegido (el más cercano visible). 0 si no hay. */
    public int getTrenchTagId() {
        return trench_tag_id;
    }

    /** Yaw (grados) del tag de Trench elegido (con offset de cámara). */
    public double getTrenchYawDeg() {
        return trench_yaw_deg;
    }

    /** Distancia (m) al tag de Trench elegido (norma 3D). */
    public double getTrenchDistanceM() {
        return trench_distance_m;
    }

    /** X (m) forward (+) hacia el tag de Trench elegido. */
    public double getTrenchX_M() {
        return trench_x_m;
    }

    /** Y (m) left (+) hacia el tag de Trench elegido. */
    public double getTrenchY_M() {
        return trench_y_m;
    }

    private static boolean isTrenchId(int id) {
        for (int trenchId : kTrenchTagIds) {
            if (id == trenchId) return true;
        }
        return false;
    }

    private static boolean isAutoAimId(int id) {
        for (int autoAimId : kAutoAimTagIds) {
            if (id == autoAimId) return true;
        }
        return false;
    }


    @Override
    public void periodic() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        has_trench_tag = false;
        trench_tag_id = 0;
        trench_yaw_deg = 0;
        trench_distance_m = 0;
        trench_x_m = 0;
        trench_y_m = 0;

        has_autoaim_tag = false;
        autoaim_tag_id = 0;
        autoaim_yaw_deg = 0;
        autoaim_distance_m = 0;

        if (result.hasTargets()) {
            PhotonTrackedTarget bestTrenchTarget = null;
            double bestTrenchDistance = Double.POSITIVE_INFINITY;
            PhotonTrackedTarget bestAutoAimTarget = null;
            double bestAutoAimDistance = Double.POSITIVE_INFINITY;
            for (PhotonTrackedTarget t : result.getTargets()) {
                // Trench selection: choose the closest Trench tag.
                int id = t.getFiducialId();
                if (isTrenchId(id)) {
                    double x = t.getBestCameraToTarget().getX();
                    double y = t.getBestCameraToTarget().getY();
                    double z = t.getBestCameraToTarget().getZ();
                    double d = Math.sqrt((x * x) + (y * y) + (z * z));
                    if (d < bestTrenchDistance) {
                        bestTrenchDistance = d;
                        bestTrenchTarget = t;
                    }
                }

                if (isAutoAimId(id)) {
                    double x = t.getBestCameraToTarget().getX();
                    double y = t.getBestCameraToTarget().getY();
                    double z = t.getBestCameraToTarget().getZ();
                    double d = Math.sqrt((x * x) + (y * y) + (z * z));
                    if (d < bestAutoAimDistance) {
                        bestAutoAimDistance = d;
                        bestAutoAimTarget = t;
                    }
                }
            }

            if (bestTrenchTarget != null) {
                has_trench_tag = true;
                trench_tag_id = bestTrenchTarget.getFiducialId();
                double rawYaw = bestTrenchTarget.getYaw();
                trench_yaw_deg = rawYaw + kCameraYawOffsetDeg;
                trench_x_m = bestTrenchTarget.getBestCameraToTarget().getX();
                trench_y_m = bestTrenchTarget.getBestCameraToTarget().getY();
                double z = bestTrenchTarget.getBestCameraToTarget().getZ();
                trench_distance_m = Math.sqrt((trench_x_m * trench_x_m) + (trench_y_m * trench_y_m) + (z * z));

                SmartDashboard.putNumber("Vision/Trench/TagId", trench_tag_id);
                SmartDashboard.putNumber("Vision/Trench/YawRawDeg", rawYaw);
                SmartDashboard.putNumber("Vision/Trench/YawDeg", trench_yaw_deg);
                SmartDashboard.putNumber("Vision/Trench/DistanceM", trench_distance_m);
                SmartDashboard.putNumber("Vision/Trench/X", trench_x_m);
                SmartDashboard.putNumber("Vision/Trench/Y", trench_y_m);
            }

            if (bestAutoAimTarget != null) {
                has_autoaim_tag = true;
                autoaim_tag_id = bestAutoAimTarget.getFiducialId();
                double rawYaw = bestAutoAimTarget.getYaw();
                autoaim_yaw_deg = rawYaw + kCameraYawOffsetDeg;
                autoaim_distance_m = bestAutoAimDistance;

                SmartDashboard.putNumber("Vision/AutoAimYawRawDeg", rawYaw);
                SmartDashboard.putNumber("Vision/AutoAimYawOffsetDeg", kCameraYawOffsetDeg);
            }

            SmartDashboard.putBoolean("Vision/HasTargets", true);
            SmartDashboard.putBoolean("Vision/Trench/HasTag", has_trench_tag);
            SmartDashboard.putBoolean("Vision/HasAutoAimTag", has_autoaim_tag);
            SmartDashboard.putNumber("Vision/AutoAimTagId", autoaim_tag_id);
            SmartDashboard.putNumber("Vision/AutoAimYawDeg", autoaim_yaw_deg);
            SmartDashboard.putNumber("Vision/AutoAimDistanceM", autoaim_distance_m);
        } else {
            SmartDashboard.putBoolean("Vision/HasTargets", false);
            SmartDashboard.putBoolean("Vision/Trench/HasTag", false);
            SmartDashboard.putNumber("Vision/Trench/TagId", 0);
            SmartDashboard.putNumber("Vision/Trench/YawRawDeg", 0);
            SmartDashboard.putNumber("Vision/Trench/YawDeg", 0);
            SmartDashboard.putNumber("Vision/Trench/DistanceM", 0);
            SmartDashboard.putNumber("Vision/Trench/X", 0);
            SmartDashboard.putNumber("Vision/Trench/Y", 0);
            SmartDashboard.putBoolean("Vision/HasAutoAimTag", false);
            SmartDashboard.putNumber("Vision/AutoAimTagId", 0);
            SmartDashboard.putNumber("Vision/AutoAimYawDeg", 0);
            SmartDashboard.putNumber("Vision/AutoAimDistanceM", 0);
            SmartDashboard.putNumber("Vision/AutoAimYawRawDeg", 0);
            SmartDashboard.putNumber("Vision/AutoAimYawOffsetDeg", kCameraYawOffsetDeg);
        }
    }


    
}
