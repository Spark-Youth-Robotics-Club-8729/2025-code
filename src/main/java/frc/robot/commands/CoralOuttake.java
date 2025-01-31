package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants.ClawWheelsConstants;

/** Spins the bottom wheel to shoot coral. */
public class ShootCoralCommand extends CommandBase {
    private final ClawWheelsSubsystem clawWheels;

    public ShootCoralCommand(ClawWheelsSubsystem subsystem) {
        clawWheels = subsystem;
        addRequirements(clawWheels);
    }

    @Override
    public void initialize() {
        clawWheels.spinBottomWheel(ClawWheelsConstants.SHOOT_CORAL_SPEED);
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
