package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawWheelsConstants;

/** Claw wheels subsystem for intake and outtake. */
public class ClawWheelsSubsystem extends SubsystemBase {
    private final SparkMax topWheelMotor; // Algae intake
    private final SparkMax bottomWheelMotor; // Coral & algae outtake

    /** Initializes motors. */
    public ClawWheelsSubsystem() {
        topWheelMotor = new SparkMax(ClawWheelsConstants.kTopWheelMotorID, MotorType.kBrushless);
        bottomWheelMotor = new SparkMax(ClawWheelsConstants.kBottomWheelMotorID, MotorType.kBrushless);
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
