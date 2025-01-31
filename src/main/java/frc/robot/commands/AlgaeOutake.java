package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants.ClawWheelsConstants;

/** Spins both wheels to shoot algae. */
public class ShootAlgaeCommand extends CommandBase {
    private final ClawWheelsSubsystem clawWheels;

    public ShootAlgaeCommand(ClawWheelsSubsystem subsystem) {
        clawWheels = subsystem;
        addRequirements(clawWheels);
    }

    @Override
    public void initialize() {
        clawWheels.spinTopWheel(ClawWheelsConstants.SHOOT_ALGAE_SPEED);
        clawWheels.spinBottomWheel(ClawWheelsConstants.SHOOT_ALGAE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        clawWheels.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
