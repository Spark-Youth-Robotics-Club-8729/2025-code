package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera; // Change camera name later
    private final PIDController pidControllerSteering;
    private final PIDController pidControllerStrafe;

    public VisionSubsystem() { // Constructor
        camera = new PhotonCamera(kCameraName);
        pidControllerSteering = new PIDController(VisionConstants.kP_Steering, VisionConstants.kI_Steering, VisionConstants.kD_Steering);
        pidControllerSteering.setTolerance(0.1);
        pidControllerStrafe = new PIDController(VisionConstants.kP_Strafing, VisionConstants.kI_Strafing, VisionConstants.kD_Strafing);
        pidControllerStrafe.setTolerance(0.1);
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

    public List<Double> getTagPose(List<Integer> aprilTagIDs) {
        PhotonPipelineResult result = camera.getLatestResult();
        List<Double> listy = new ArrayList<>();

        if (result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());

            for (PhotonTrackedTarget target : result.getTargets()) {
                for (int aprilTagID : aprilTagIDs) {
                    if (target.getFiducialId() == aprilTagID) {
                        Transform3d tagPose = target.getBestCameraToTarget();
                        SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
                        SmartDashboard.putNumber("Tag Pose Z", tagPose.getRotation().getZ());
                        SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
                        listy.add(tagPose.getRotation().getZ());
                        listy.add(tagPose.getY());
                        return listy;
                    }
                }
            }
        }

        listy.add(0.0);
        listy.add(0.0);
        return listy;
    }

    public List<Double> align(double yOffset) {
        List<Double> result = getTagPose(VisionConstants.kAprilTagIds);
        double offsetAngle = result.get(0);
        double yStrafing = result.get(1) - yOffset;
        double outputAngle = pidControllerSteering.calculate(Math.abs(offsetAngle), 0.0);
        double outputStrafing = pidControllerStrafe.calculate(yStrafing, 0.0);
        List<Double> listy = new ArrayList<>();
        listy.add(outputAngle);
        listy.add(outputStrafing);
        SmartDashboard.putNumber("PID Output 0", listy.get(0));
        SmartDashboard.putNumber("PID Output 1", listy.get(1));

        return listy;
    }

    public boolean AngleisAtSetpoint() {
        return pidControllerSteering.atSetpoint();
    }

    public boolean YisAtSetpoint() {
        return pidControllerStrafe.atSetpoint();
    }

    public void resetPID() {
        pidControllerSteering.reset();
        pidControllerStrafe.reset();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("getAngleOffset Number", getAngleOffset(VisionConstants.kAprilTagIds));
    }

    
}