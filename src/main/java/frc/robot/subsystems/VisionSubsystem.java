package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

// Links:
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // https://docs.photonvision.org/en/v2025.0.0-beta-8/docs/programming/photonlib/getting-target-data.html

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera; // Change camera name later

    public VisionSubsystem() { // Constructor
        camera = new PhotonCamera(kCameraName);
    }

    // https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html#knowledge-and-equipment-needed
    public double getAngleOffset(int aprilTagID) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
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
        }
        return 0.0; // If no april tag found
    }

    
}
