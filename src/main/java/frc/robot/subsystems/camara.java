package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camara extends SubsystemBase {

    public final PhotonCamera camera = new PhotonCamera("photonvision");

    private double x_distance;
    private double y_distance;
    private double distance_to_target;
    private double angle_to_target;



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


    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            x_distance = target.getBestCameraToTarget().getX();
            y_distance = target.getBestCameraToTarget().getY();
            distance_to_target = target.getBestCameraToTarget().getZ();
            angle_to_target = target.getYaw();

            SmartDashboard.putNumber("X Distance", x_distance);
            SmartDashboard.putNumber("Y Distance", y_distance);
            SmartDashboard.putNumber("Distance to Target", distance_to_target);
            SmartDashboard.putNumber("Angle to Target", angle_to_target);
        } else {
            x_distance = 0;
            y_distance = 0;
            distance_to_target = 0;
            angle_to_target = 0;

            SmartDashboard.putNumber("X Distance", x_distance);
            SmartDashboard.putNumber("Y Distance", y_distance);
            SmartDashboard.putNumber("Distance to Target", distance_to_target);
            SmartDashboard.putNumber("Angle to Target", angle_to_target);
        }
    }


    
}
