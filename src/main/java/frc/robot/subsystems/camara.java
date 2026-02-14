package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camara extends SubsystemBase {

    public final PhotonCamera camera = new PhotonCamera("GENERAL_WEBCAM (1)");

    private double x_distance;
    private double y_distance;
    private double distance_to_target;
    private double angle_to_target;
    private boolean has_tag1;
    private double tag1_yaw_deg;



    public double getDistance() {
        return x_distance;
    }

    public double getYDistance() {
        return y_distance;
    }

    public double getAngleToTarget() {
        return angle_to_target;
    }

    public double getDistanceToTarget() {
        return distance_to_target;
    }

    public boolean hasTag1() {
        return has_tag1;
    }

    public double getTag1YawDeg() {
        return tag1_yaw_deg;
    }


    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        has_tag1 = false;
        tag1_yaw_deg = 0;

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            PhotonTrackedTarget tag1Target = null;
            for (PhotonTrackedTarget t : result.getTargets()) {
                if (t.getFiducialId() == 1) {
                    if (tag1Target == null || Math.abs(t.getYaw()) < Math.abs(tag1Target.getYaw())) {
                        tag1Target = t;
                    }
                }
            }

            if (tag1Target != null) {
                has_tag1 = true;
                tag1_yaw_deg = tag1Target.getYaw();
            }

            x_distance = target.getBestCameraToTarget().getX();
            y_distance = target.getBestCameraToTarget().getY();
            distance_to_target = target.getBestCameraToTarget().getZ();
            angle_to_target = target.getYaw();

            SmartDashboard.putNumber("X Distance", x_distance);
            SmartDashboard.putNumber("Y Distance", y_distance);
            SmartDashboard.putNumber("Distance to Target", distance_to_target);
            SmartDashboard.putNumber("Angle to Target", angle_to_target);
            SmartDashboard.putBoolean("AutoAim/HasTag1", has_tag1);
            SmartDashboard.putNumber("AutoAim/Tag1YawDeg", tag1_yaw_deg);
        } else {
            x_distance = 0;
            y_distance = 0;
            distance_to_target = 0;
            angle_to_target = 0;

            SmartDashboard.putNumber("X Distance", x_distance);
            SmartDashboard.putNumber("Y Distance", y_distance);
            SmartDashboard.putNumber("Distance to Target", distance_to_target);
            SmartDashboard.putNumber("Angle to Target", angle_to_target);
            SmartDashboard.putBoolean("AutoAim/HasTag1", false);
            SmartDashboard.putNumber("AutoAim/Tag1YawDeg", 0);
        }
    }


    
}
