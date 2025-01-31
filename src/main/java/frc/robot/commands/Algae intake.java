package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawWheelsSubsystem;
import frc.robot.Constants.ClawWheelsConstants;

/** Spins the top wheel to intake algae. */
public class IntakeAlgaeCommand extends CommandBase {
    private final ClawWheelsSubsystem clawWheels;

    public IntakeAlgaeCommand(ClawWheelsSubsystem subsystem) {
        clawWheels = subsystem;
        addRequirements(clawWheels);
    }

    @Override
    public void initialize() {
        clawWheels.spinTopWheel(ClawWheelsConstants.INTAKE_ALGAE_SPEED);
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