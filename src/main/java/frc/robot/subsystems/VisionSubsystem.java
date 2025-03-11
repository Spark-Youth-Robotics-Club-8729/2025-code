package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera; // Change camera name later

    public VisionSubsystem() { // Constructor
        camera = new PhotonCamera(kCameraName);
    }

    
    // https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html#knowledge-and-equipment-needed
    public double getAngleOffset(int aprilTagID) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == aprilTagID) {
                    // Found target april tag id
                    double targetYaw = target.getYaw();
                    return targetYaw;
                }
            }
            
        }
        return 0.0; // If no april tag found
    }


    @Override
    public void periodic() {
        // Get the latest camera result
        PhotonPipelineResult result = camera.getLatestResult();

        // Log whether any targets are detected
        boolean hasTargets = false;
        double yaw = 0.0;

        if (result.hasTargets()) {
            hasTargets = true;

            // Get the yaw of the first detected target (you can adjust this if you want all targets)
            var target = result.getTargets().get(0);  // Assuming only interested in the first target
            yaw = target.getYaw();
            SmartDashboard.putNumber("Vision/April Tag ID", target.getFiducialId());
        }

        // Log data to SmartDashboard (or Shuffleboard)
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets);
        SmartDashboard.putNumber("Vision/Yaw", yaw);
        SmartDashboard.putNumber("Vision/Number of targets:", result.getTargets().size());

        SmartDashboard.putNumber("Vision/Timestamp", System.currentTimeMillis());
        SmartDashboard.putNumber("Vision/NumberOfTargets", result.getTargets().size());
    }

    
}