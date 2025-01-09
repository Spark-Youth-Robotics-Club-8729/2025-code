// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class VisionConstants {
    public static final String CAMERA_NAME = "photonvision_camera";

    // April Tag Layout for Pose Estimator
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final int CAMERA_ROLL = 0;
    public static final int CAMERA_PITCH = 0;
    public static final double CAMERA_YAW = Math.PI;


    // Where the camera is mounted with regards to the robot
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0, 0, 0),
      new Rotation3d(CAMERA_ROLL, CAMERA_PITCH * Math.PI/180, CAMERA_YAW));
        // Translation3d is offset from center or robot/robot origin
        // Rotation3d is rotation on forward/back axis, rotation on left/right axis, rotation on up/down axis

    // Set values for std devs to determine trustworthiness
      // Change with actual values in future
    public static final Matrix<N3, N1> SINGLE_TAG_SD = VecBuilder.fill(0, 0, 0); // Should be higher
    public static final Matrix<N3, N1> MULTI_TAG_SD = VecBuilder.fill(0, 0, 0);
  }


}
