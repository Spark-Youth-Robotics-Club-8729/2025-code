package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera; // Change camera name later

    public VisionSubsystem() { // Constructor
        camera = new PhotonCamera(kCameraName);
    }
    
    // https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html#knowledge-and-equipment-needed
    public double getAngleOffset(List<Integer> aprilTagIDs) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                for (var aprilTagID : aprilTagIDs) {
                    if (target.getFiducialId() == aprilTagID) {
                        // Found target april tag id
                        double targetYaw = target.getYaw();
                        SmartDashboard.putNumber("Camera Yaw", targetYaw);
                        return targetYaw;
                    }
                }
            }
            
        }
        return 0.0; // If no april tag found
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("getAngleOffset Number", getAngleOffset(VisionConstants.kAprilTagIds));
    }

    
}