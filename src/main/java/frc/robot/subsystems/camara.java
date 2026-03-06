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
    private static final int[] kAutoAimTagIds = new int[] {8, 5, 4, 3, 11, 9, 10, 18, 19, 20, 21, 24, 25, 26, 27};

    private boolean has_autoaim_tag;
    private int autoaim_tag_id;
    private double autoaim_yaw_deg;
    private double autoaim_distance_m;

    // =========================
    // "Trench" tags: IDs que se usan para alinear el robot en esa zona.
    // =========================
    // Nota: lo dejamos aquí para que el lector entienda que estos tags existen con un propósito de juego.
    private static final int[] kTrenchTagIds = new int[] {7, 6, 12, 1, 17, 26, 22, 23, 2};

    private boolean has_trench_tag;
    private int trench_tag_id;
    private double trench_yaw_deg;
    private double trench_distance_m;
    private double trench_x_m;
    private double trench_y_m;

    /**
     * Offset de montaje de la cámara.
     *
     * <p>Si la cámara está 10cm a la derecha del centro del robot, entonces el vector
     * camera->tag tiene un componente lateral (Y) que NO corresponde al centro del robot.
     * Compensamos eso ajustando el Y antes de calcular yaw/distancia 2D.
     */
    private static final double kCameraMountRightOffsetM = 0.0; // +derecha (m)

    /**
     * Offset horizontal de montaje de la cámara (grados) para compensar yaw si el lente
     * no está perfectamente alineado.
     */
    private static final double kCameraYawOffsetDeg = 0.0;

    private static final String kCameraRightOffsetKey = "Vision/Camera/MountRightOffsetM";

    private boolean m_rightOffsetDashboardInitialized = false;

    // Publicaciones: mantener consistencia (NetworkTables sólo muestra/crea keys que se publican).
    private static final String kVisionHasTargetsKey = "Vision/HasTargets";
    private static final String kVisionHasAutoAimTagKey = "Vision/HasAutoAimTag";
    private static final String kVisionAutoAimTagIdKey = "Vision/AutoAimTagId";
    private static final String kVisionAutoAimYawDegKey = "Vision/AutoAimYawDeg";
    private static final String kVisionAutoAimDistanceMKey = "Vision/AutoAimDistanceM";
    private static final String kVisionAutoAimYawRawDegKey = "Vision/AutoAimYawRawDeg";
    private static final String kVisionAutoAimYawOffsetDegKey = "Vision/AutoAimYawOffsetDeg";
    private static final String kVisionAutoAimRightOffsetMKey = "Vision/AutoAimRightOffsetM";
    private static final String kVisionAutoAimRawYKey = "Vision/AutoAim/RawY";
    private static final String kVisionAutoAimCorrectedYKey = "Vision/AutoAim/CorrectedY";

    private static final String kVisionTrenchHasTagKey = "Vision/Trench/HasTag";
    private static final String kVisionTrenchTagIdKey = "Vision/Trench/TagId";
    private static final String kVisionTrenchYawRawDegKey = "Vision/Trench/YawRawDeg";
    private static final String kVisionTrenchYawDegKey = "Vision/Trench/YawDeg";
    private static final String kVisionTrenchDistanceMKey = "Vision/Trench/DistanceM";
    private static final String kVisionTrenchXKey = "Vision/Trench/X";
    private static final String kVisionTrenchYKey = "Vision/Trench/Y";

    private static final String kVisionCameraConnectedKey = "Vision/Camera/Connected";

    /**
     * Resultado de geometría usando el vector corregido (como si midiera desde el centro del robot).
     * yawDeg: +izquierda (si Y+ es izquierda), -derecha.
     */
    private static final class CorrectedTarget {
        final double x;
        final double y;
        final double distance;
        final double yawDeg;

        CorrectedTarget(double x, double y, double distance, double yawDeg) {
            this.x = x;
            this.y = y;
            this.distance = distance;
            this.yawDeg = yawDeg;
        }
    }

    private static CorrectedTarget computeCorrectedTarget(PhotonTrackedTarget target, double mountRightOffsetM) {
        double x = target.getBestCameraToTarget().getX();
        // Corregir el componente lateral porque la cámara no está en el centro del robot.
        // Convencion esperada: +mountRightOffsetM significa "camara montada a la derecha".
        // Para obtener el vector desde el centro del robot hacia el tag,
        // sumamos el offset (en vez de restarlo) para no invertir el yaw.
        double rawY = target.getBestCameraToTarget().getY();
        double y = rawY + mountRightOffsetM;
        double z = target.getBestCameraToTarget().getZ();
        double distance = Math.sqrt((x * x) + (y * y) + (z * z));
        double yawDeg = Math.toDegrees(Math.atan2(y, x));
        // Telemetria: esto solo vive durante el ciclo donde se computa el target.
        SmartDashboard.putNumber(kVisionAutoAimRawYKey, rawY);
        SmartDashboard.putNumber(kVisionAutoAimCorrectedYKey, y);
        return new CorrectedTarget(x, y, distance, yawDeg);
    }

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
        // Publicar/leer offset para poder tunear en Shuffleboard si quieres.
        // Importante: NO publicar el default cada ciclo, porque pisas el valor que el usuario cambie.
        if (!m_rightOffsetDashboardInitialized) {
            SmartDashboard.putNumber(kCameraRightOffsetKey, kCameraMountRightOffsetM);
            m_rightOffsetDashboardInitialized = true;
        }
        final double rightOffsetM = SmartDashboard.getNumber(kCameraRightOffsetKey, kCameraMountRightOffsetM);

    // Publicar defaults SIEMPRE para que Shuffleboard no pierda keys.
        SmartDashboard.putBoolean(kVisionHasTargetsKey, false);
        SmartDashboard.putBoolean(kVisionTrenchHasTagKey, false);
        SmartDashboard.putNumber(kVisionTrenchTagIdKey, 0);
        SmartDashboard.putNumber(kVisionTrenchYawRawDegKey, 0);
        SmartDashboard.putNumber(kVisionTrenchYawDegKey, 0);
        SmartDashboard.putNumber(kVisionTrenchDistanceMKey, 0);
        SmartDashboard.putNumber(kVisionTrenchXKey, 0);
        SmartDashboard.putNumber(kVisionTrenchYKey, 0);

        SmartDashboard.putBoolean(kVisionHasAutoAimTagKey, false);
        SmartDashboard.putNumber(kVisionAutoAimTagIdKey, 0);
        SmartDashboard.putNumber(kVisionAutoAimYawDegKey, 0);
        SmartDashboard.putNumber(kVisionAutoAimDistanceMKey, 0);
        SmartDashboard.putNumber(kVisionAutoAimYawRawDegKey, 0);
        SmartDashboard.putNumber(kVisionAutoAimYawOffsetDegKey, kCameraYawOffsetDeg);
        SmartDashboard.putNumber(kVisionAutoAimRightOffsetMKey, rightOffsetM);

        boolean cameraConnected = m_camera.isConnected();
        SmartDashboard.putBoolean(kVisionCameraConnectedKey, cameraConnected);

        if (!cameraConnected) {
            // Si no hay cámara, no intentes leer resultados (evita errores y deja todo en default).
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
            return;
        }

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
                    double d = computeCorrectedTarget(t, rightOffsetM).distance;
                    if (d < bestTrenchDistance) {
                        bestTrenchDistance = d;
                        bestTrenchTarget = t;
                    }
                }

                if (isAutoAimId(id)) {
                    double d = computeCorrectedTarget(t, rightOffsetM).distance;
                    if (d < bestAutoAimDistance) {
                        bestAutoAimDistance = d;
                        bestAutoAimTarget = t;
                    }
                }
            }

            if (bestTrenchTarget != null) {
                has_trench_tag = true;
                trench_tag_id = bestTrenchTarget.getFiducialId();
                CorrectedTarget ct = computeCorrectedTarget(bestTrenchTarget, rightOffsetM);
                trench_x_m = ct.x;
                trench_y_m = ct.y;
                trench_distance_m = ct.distance;
                double rawYaw = ct.yawDeg;
                trench_yaw_deg = rawYaw + kCameraYawOffsetDeg;

                SmartDashboard.putBoolean(kVisionTrenchHasTagKey, true);
                SmartDashboard.putNumber(kVisionTrenchTagIdKey, trench_tag_id);
                SmartDashboard.putNumber(kVisionTrenchYawRawDegKey, rawYaw);
                SmartDashboard.putNumber(kVisionTrenchYawDegKey, trench_yaw_deg);
                SmartDashboard.putNumber(kVisionTrenchDistanceMKey, trench_distance_m);
                SmartDashboard.putNumber(kVisionTrenchXKey, trench_x_m);
                SmartDashboard.putNumber(kVisionTrenchYKey, trench_y_m);
            }

            if (bestAutoAimTarget != null) {
                has_autoaim_tag = true;
                autoaim_tag_id = bestAutoAimTarget.getFiducialId();
                CorrectedTarget ct = computeCorrectedTarget(bestAutoAimTarget, rightOffsetM);
                autoaim_distance_m = ct.distance;
                double rawYaw = ct.yawDeg;
                autoaim_yaw_deg = rawYaw + kCameraYawOffsetDeg;

                SmartDashboard.putBoolean(kVisionHasAutoAimTagKey, true);
                SmartDashboard.putNumber(kVisionAutoAimTagIdKey, autoaim_tag_id);
                SmartDashboard.putNumber(kVisionAutoAimYawDegKey, autoaim_yaw_deg);
                SmartDashboard.putNumber(kVisionAutoAimDistanceMKey, autoaim_distance_m);
                SmartDashboard.putNumber(kVisionAutoAimYawRawDegKey, rawYaw);
                SmartDashboard.putNumber(kVisionAutoAimYawOffsetDegKey, kCameraYawOffsetDeg);
                SmartDashboard.putNumber(kVisionAutoAimRightOffsetMKey, rightOffsetM);
            }

            SmartDashboard.putBoolean(kVisionHasTargetsKey, true);
        } else {
            // Defaults ya publicados arriba.
        }
    }


    
}
