package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawWheelsConstants;

/** Claw wheels subsystem for intake and outtake. */
public class ClawWheelsSubsystem extends SubsystemBase {
    private final CANSparkMax topWheelMotor; // Algae intake
    private final CANSparkMax bottomWheelMotor; // Coral & algae outtake

    /** Initializes motors. */
    public ClawWheelsSubsystem(int topWheelMotorID, int bottomWheelMotorID) {
        topWheelMotor = new CANSparkMax(topWheelMotorID, MotorType.kBrushless);
        bottomWheelMotor = new CANSparkMax(bottomWheelMotorID, MotorType.kBrushless);
    }

    /** Spins top wheel. */
    public void spinTopWheel(double speed) {
        topWheelMotor.set(speed);
    }

    /** Spins bottom wheel. */
    public void spinBottomWheel(double speed) {
        bottomWheelMotor.set(speed);
    }

    /** Stops all motors. */
    public void stopAll() {
        topWheelMotor.stopMotor();
        bottomWheelMotor.stopMotor();
    }
}
