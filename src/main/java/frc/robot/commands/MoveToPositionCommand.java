package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.function.Supplier;

public class MoveToPositionCommand extends Command {
    private final DriveSubsystem m_DriveSubsystem;
    private final Pose2d targetPose;
    private SwerveControllerCommand m_swerveControllerCommand;

    // Define PID controllers for x, y, and theta
    private final PIDController xController = new PIDController(0.01, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.01, 0.0, 0.0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(0.15, 0.0, 0.0, 
    new TrapezoidProfile.Constraints(Math.PI, Math.PI/2));

    public MoveToPositionCommand(DriveSubsystem d, Pose2d t) {
        m_DriveSubsystem = d;
        targetPose = t;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        // Get the current pose of the robot
        Pose2d currentPose = m_DriveSubsystem.getPoseZero();

        // Configure the trajectory
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                0.25, 0.1)
                .setKinematics(DriveConstants.kDriveKinematics);

        // Generate the trajectory from the current pose to the target pose
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose, // Start pose
                new ArrayList<>(), // No intermediate waypoints
                targetPose, // End pose
                trajectoryConfig);

        // Create a SwerveControllerCommand to follow the trajectory
        m_swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_DriveSubsystem::getPoseZero, // Use the getPose method as a Supplier<Pose2d>
                DriveConstants.kDriveKinematics, // Swerve drive kinematics
                xController, // PID controller for x
                yController, // PID controller for y
                thetaController, // PID controller for theta
                m_DriveSubsystem::setModuleStates, // Method reference to apply the module states
                m_DriveSubsystem
                );

        // Reset odometry to the starting position of the trajectory
        m_DriveSubsystem.resetOdometry(trajectory.getInitialPose());
        m_swerveControllerCommand.execute();


        SmartDashboard.putData("TRAJECTORY", m_swerveControllerCommand);

    }

    @Override
    public void execute() {
        // Execute the SwerveControllerCommand, which will control the robot to follow the trajectory
    }

    @Override
    public boolean isFinished() {
        // The command finishes once the trajectory is complete
        return m_swerveControllerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command finishes or is interrupted
        m_DriveSubsystem.stop();
    }
}
