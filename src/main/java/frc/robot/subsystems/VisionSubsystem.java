package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera camera; // Change camera name later
    private double lastEstTimestamp = 0;
    private final PhotonPoseEstimator photonEstimator;
    private final PIDController pidControllerSteering;
    private final PIDController pidControllerYStrafe;
    private final PIDController pidControllerXStrafe;
    AprilTagFieldLayout fieldPoseLayout;


    public VisionSubsystem() { // Constructor
        camera = new PhotonCamera(kCameraName);
        fieldPoseLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        pidControllerSteering = new PIDController(VisionConstants.kP_Steering, VisionConstants.kI_Steering, VisionConstants.kD_Steering);
        pidControllerSteering.setTolerance(0.1);
        pidControllerYStrafe = new PIDController(VisionConstants.kP_Strafing, VisionConstants.kI_Strafing, VisionConstants.kD_Strafing);
        pidControllerYStrafe.setTolerance(0.05);
        pidControllerXStrafe = new PIDController(VisionConstants.kP_Strafing, VisionConstants.kI_Strafing, VisionConstants.kD_Strafing);
        pidControllerXStrafe.setTolerance(0.05);
        photonEstimator = new PhotonPoseEstimator(
                VisionConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update(getLatestResult());
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult)
            lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
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

    public List<Double> getTagPose(List<Integer> aprilTagIDs, double yOffset) {
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
                        listy.add(tagPose.getX());
                        return listy;
                    }
                }
            }
        }

        listy.add(0.0);
        listy.add(0.0);
        return listy;
    }

    public List<Double> getTagList(List<Integer> aprilTagIDs, double yOffset) {
        PhotonPipelineResult result = camera.getLatestResult();
        List<Double> listy = new ArrayList<>();

        // Add yoffset
        if (result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());

            for (PhotonTrackedTarget target : result.getTargets()) {
                for (int aprilTagID : aprilTagIDs) {
                    if (target.getFiducialId() == aprilTagID) {
                        Transform3d tagPose = target.getBestCameraToTarget();
                        SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
                        SmartDashboard.putNumber("Tag Pose Z", tagPose.getRotation().getZ());
                        SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());

                        double value = tagPose.getRotation().getZ();
                        double y = tagPose.getY();

                        if (value > 0) {
                            value = -(Math.abs(value)-(Math.PI/2));
                            y = -y;
                        } else {
                            value = (Math.abs(value)-(Math.PI/2));
                            y = -y;
                        }

                        double x = tagPose.getX()*Math.cos(value);

                        listy.add(value);
                        listy.add(x);
                        listy.add(y);


                        return listy;
                    }
                }
            }
        }

        listy.add(0.0);
        listy.add(0.0);
        listy.add(0.0);
        return listy;
    }

    public List<Double> alignPID(double yOffset) {
        List<Double> result = getTagList(VisionConstants.kAprilTagIds, yOffset);
        
        double outputAngle = pidControllerSteering.calculate(result.get(0), 0.0);
        double outputXStrafing = pidControllerXStrafe.calculate(result.get(1), 0.20);
        double outputYStrafing = pidControllerYStrafe.calculate(result.get(2), 0.0);
        List<Double> listy = new ArrayList<>();
        listy.add(outputAngle);
        listy.add(outputXStrafing);
        listy.add(outputYStrafing);
        SmartDashboard.putNumber("PID Output 0", listy.get(0));
        SmartDashboard.putNumber("PID Output 1", listy.get(1));
        SmartDashboard.putNumber("PID Output 2", listy.get(2));
        SmartDashboard.putNumber("XComponent", result.get(1));
        SmartDashboard.putNumber("YComponent", result.get(2));
        SmartDashboard.putNumber("OffsetAngle", result.get(0));

        return listy;
    }

    public List<Double> getTagPoseTrajectoryPID(List<Integer> aprilTagIDs, double yOffset) {
        PhotonPipelineResult result = camera.getLatestResult();
        List<Double> listy = new ArrayList<>();
        

        if (result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());

            for (PhotonTrackedTarget target : result.getTargets()) {
                for (int aprilTagID : aprilTagIDs) {
                    if (target.getFiducialId() == aprilTagID) {
                        Transform3d tagPose = target.getBestCameraToTarget();
                        double value = tagPose.getRotation().getZ();
                        if (value > 0) {
                            value = -(Math.abs(value)-(Math.PI/2));
                        } else {
                            value = (Math.abs(value)-(Math.PI/2));
                        }
                        //Pose2d pose = new Pose2d(Math.abs(tagPose.getX()*Math.cos(value)), tagPose.getY(), new Rotation2d(value));
                        listy.add(tagPose.getX()*Math.cos(value));
                        listy.add(tagPose.getY());
                        listy.add(value);
                        return listy;

                    }
                }
            }
        }

        listy.add(0.0);
        listy.add(0.0);
        listy.add(0.0);

        return listy;

    }

    public Pose2d returnAprilTagPose(int aprilTag) {
        Pose3d tagPose = fieldPoseLayout.getTagPose(aprilTag).get();
        double lx = tagPose.getX();
        double rx = tagPose.getX();
        double ly = tagPose.getY();
        double ry = tagPose.getY();
        double angle = tagPose.getRotation().getZ();
        SmartDashboard.putNumber("TagDETECT X", lx);
        SmartDashboard.putNumber("TagDETECT Y", ly);
        SmartDashboard.putNumber("TagDETECT Z", angle*180/Math.PI);

        angle += Math.PI;
        Pose2d pose;
        pose = new Pose2d(lx, ly, new Rotation2d(angle));
        return pose;
    }

    public Pose2d getTagPose2dConditionals(List<Integer> aprilTagIDs, boolean left) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());

            for (PhotonTrackedTarget target : result.getTargets()) {
                for (int aprilTagID : aprilTagIDs) {
                    if (target.getFiducialId() == aprilTagID) {
                        int id = target.getFiducialId(); //0.164338
                        Pose3d tagPose = fieldPoseLayout.getTagPose(id).get();
                        double lx = tagPose.getX();
                        double rx = tagPose.getX();
                        double ly = tagPose.getY();
                        double ry = tagPose.getY();
                        double angle = tagPose.getRotation().getZ();
                        SmartDashboard.putNumber("TagDETECT X", lx);
                        SmartDashboard.putNumber("TagDETECT Y", ly);
                        SmartDashboard.putNumber("TagDETECT Z", angle*180/Math.PI);

                        SmartDashboard.putNumber("MODIFIED Z", angle);

                        angle += Math.PI;



                        // if (id==7 || id==21) {
                        //     ly -= 0.164338;
                        //     ry += 0.164338;
                        // } else if (id==8 || id==20) {
                        //     ly -= 0.164338*Math.sin(Math.PI/3);
                        //     ry += 0.164338*Math.cos(Math.PI/3);
                        //     lx += 0.164338*Math.sin(Math.PI/3);
                        //     rx -= 0.164338*Math.cos(Math.PI/3);
                        // } else if (id==9 || id==19) {
                        //     ly += 0.164338*Math.sin(Math.PI/3);
                        //     ry -= 0.164338*Math.cos(Math.PI/3);
                        //     lx += 0.164338*Math.sin(Math.PI/3);
                        //     rx -= 0.164338*Math.cos(Math.PI/3);
                        // } else if (id==10 || id==18) {
                        //     ly += 0.164338;
                        //     ry -= 0.164338;
                        // } else if (id==11 || id==17) {
                        //     ly += 0.164338*Math.sin(Math.PI/3);
                        //     ry -= 0.164338*Math.cos(Math.PI/3);
                        //     lx -= 0.164338*Math.sin(Math.PI/3);
                        //     rx += 0.164338*Math.cos(Math.PI/3);
                        // } else if (id==6 || id==22) {
                        //     ly -= 0.164338*Math.sin(Math.PI/3);
                        //     ry += 0.164338*Math.cos(Math.PI/3);
                        //     lx -= 0.164338*Math.sin(Math.PI/3);
                        //     rx += 0.164338*Math.cos(Math.PI/3);
                        // }

                        // Pose2d pose;
                        // if (left) {
                        //     pose = new Pose2d(lx, ly, new Rotation2d(angle));
                            
                        // } else {
                        //     pose = new Pose2d(rx, ry, new Rotation2d(angle));
                        // }

                        Pose2d pose;
                        pose = new Pose2d(lx, ly, new Rotation2d(angle));
                        return pose;
                    }
                }
            }
        }

        return null;
    }

    public Pose2d getTagPoseTrajectory(List<Integer> aprilTagIDs, double yOffset) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            SmartDashboard.putBoolean("Vision/HasTargets", result.hasTargets());

            for (PhotonTrackedTarget target : result.getTargets()) {
                for (int aprilTagID : aprilTagIDs) {
                    if (target.getFiducialId() == aprilTagID) {
                        Transform3d tagPose = target.getBestCameraToTarget();
                        double value = tagPose.getRotation().getZ();
                        if (value > 0) {
                            value = -(Math.abs(value)-(Math.PI/2));
                        } else {
                            value = (Math.abs(value)-(Math.PI/2));
                        }
                        Pose2d pose = new Pose2d(Math.abs(tagPose.getX()*Math.cos(value)), tagPose.getY(), new Rotation2d(value));
                        return pose;

                    }
                }
            }
        }

        return null;
    }


    public List<Double> applyPIDMarch19Code(double yOffset) {
        List<Double> result = getTagPoseTrajectoryPID(VisionConstants.kAprilTagIds, yOffset);

        double outputXStrafing = pidControllerXStrafe.calculate(result.get(0));
        double outputYStrafing = pidControllerYStrafe.calculate(result.get(1));
        double outputAngle = pidControllerSteering.calculate(result.get(2));


        List<Double> listy = new ArrayList<>();
        listy.add(outputYStrafing);
        listy.add(outputXStrafing);
        listy.add(outputAngle);


        return listy;
    }

    public List<Double> align(double yOffset) {
        List<Double> result = getTagPose(VisionConstants.kAprilTagIds, yOffset);
        boolean negative = result.get(0) < 0 ? true : false;
        double offsetAngle = Math.abs(result.get(0)) - Math.PI/2;
        double xStrafing = offsetAngle*Math.cos(offsetAngle);
        double yStrafing = offsetAngle*Math.sin(offsetAngle) - yOffset;

        if (negative) {
            yStrafing = -yStrafing;
        }

        double outputAngle = pidControllerSteering.calculate(offsetAngle*result.get(0)/Math.abs(result.get(0)), 0.0);
        double outputXStrafing = pidControllerXStrafe.calculate(xStrafing, 0.20);
        double outputYStrafing = pidControllerYStrafe.calculate(yStrafing, 0.0);
        List<Double> listy = new ArrayList<>();
        listy.add(outputAngle);
        listy.add(outputYStrafing);
        listy.add(outputXStrafing);
        SmartDashboard.putNumber("PID Output 0", listy.get(0));
        SmartDashboard.putNumber("PID Output 1", listy.get(1));
        SmartDashboard.putNumber("PID Output 2", listy.get(2));
        SmartDashboard.putNumber("XComponent", xStrafing);
        SmartDashboard.putNumber("YComponent", yStrafing);
        SmartDashboard.putNumber("OffsetAngle", offsetAngle);

        return listy;
    }

    public boolean AngleisAtSetpoint() {
        return (Math.abs(pidControllerSteering.getError()) < 0.05);
    }

    public boolean YisAtSetpoint() {
        return pidControllerYStrafe.atSetpoint();
    }

    public boolean XisAtSetpoint() {
        return pidControllerXStrafe.atSetpoint();
    }

    public boolean isAtSetpoint() {
        return AngleisAtSetpoint() && YisAtSetpoint() && XisAtSetpoint();
    }

    public void resetPID() {
        pidControllerSteering.reset();
        pidControllerYStrafe.reset();
        pidControllerXStrafe.reset();

    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("getAngleOffset Number", getAngleOffset(VisionConstants.kAprilTagIds));
        getTagPose2dConditionals(VisionConstants.kAprilTagIds, isAtSetpoint());
    }

    
}