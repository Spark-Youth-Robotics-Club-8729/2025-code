package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.AbstractMap;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Links:
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // https://docs.photonvision.org/en/v2025.0.0-beta-8/docs/programming/photonlib/getting-target-data.html

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera; // Change camera name later
    private final PhotonPoseEstimator poseEstimator;
    private double lastEstTime;

    public VisionSubsystem() { // Constructor
        camera = new PhotonCamera(kCameraName);
        
        poseEstimator = new PhotonPoseEstimator(
            TAG_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            kRobotToCam
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // What to use if only 1 tag found
    }

    // Returns an estimated pose, estimate timestamp, and targets used for estimation.
    public AbstractMap.SimpleEntry<Optional<EstimatedRobotPose>, Pose3d> getEstimatedGlobalPose() {
        // Get pipline result
        var result = camera.getLatestResult();

        // Update pose estimator with result
        Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);
        // Only new results when new time
        double latestTime = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTime - lastEstTime) > 1e-5;
        if (newResult)
            lastEstTime = latestTime;

        Pose3d estimatedPose = poseEstimator.getReferencePose();
        return new AbstractMap.SimpleEntry<>(visionEst, estimatedPose);
    }

    // Returns standard deviations (essentially trustworthiness) of the estimated pose (to be used alongside swerve to align robot)
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagSD;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;

        // Loop through all detected targets
        for (var target: targets) {
            // Get pose (position + orientation)
            var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            // Distance between tag and estimated robot pose
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0)
            return estStdDevs;
        // Average distance
        avgDist /= numTags;
        // If more than 1 tag -> need to use a more trusting SD
        if (numTags > 1)
            estStdDevs = kMultiTagSD;
        // If only 1 tag and distance is very far (untrustworthy)
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    // Calculate offset of robot to a specific april tag (x, y and angle)
    public Optional<Pose2d> calculateOffsetToTag(int targetTagId) {
        var result = camera.getLatestResult();
        var targets = result.getTargets();

        // Find the target April Tag from the seen targets
        for (var target: targets) {
            if (target.getFiducialId() == targetTagId) {
                // Get the pose of the tag
                var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    AbstractMap.SimpleEntry<Optional<EstimatedRobotPose>, Pose3d> poseEntry = getEstimatedGlobalPose();

                    // Extract
                    Optional<EstimatedRobotPose> robotPose = poseEntry.getKey();
                    Pose3d robot3dPose = poseEntry.getValue();

                    if (robotPose.isPresent()) {
                        // Convert Pose3d to Pose2d 
                        Pose2d robot2dPose = new Pose2d(robot3dPose.getX(), robot3dPose.getY(), robot3dPose.getRotation().toRotation2d());
                        
                        // Calculate the offset (difference in position)
                        Pose2d tagPosition = tagPose.get().toPose2d();
                        double offsetX = robot2dPose.getX() - tagPosition.getX();
                        double offsetY = robot2dPose.getY() - tagPosition.getY();
                        double offsetAngle = robot2dPose.getRotation().getDegrees() - tagPosition.getRotation().getDegrees();

                        // Return the offset as a new Pose2d
                        return Optional.of(new Pose2d(offsetX, offsetY, Rotation2d.fromDegrees(offsetAngle)));
                    }
                }
            }
        }
        // Return empty if tag not found
        return Optional.empty();
    }



    
}
