// controls elevator up and down
// NOTE: add button bindings that enter speed positive when pressing one button and speed negative when pressing another button
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KrakenMotorSubsystem;

public class Elevator extends CommandBase {
    private final KrakenMotorSubsystem elevator;
    private final double speed;


    public Elevator(KrakenMotorSubsystem elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    // when button pressed rotate
    @Override
    public void execute() {
        elevator.setMotorSpeed(speed);
    }

    // when button not pressed stop rotating
    @Override
    public void end(boolean interrupted) {
        elevator.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
}
