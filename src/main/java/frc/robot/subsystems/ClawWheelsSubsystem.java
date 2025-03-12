package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawWheelsConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/digital-inputs-software.html -> for IR Break Beam Sensor

/* Claw wheels subsystem for intake and outtake. */
public class ClawWheelsSubsystem extends SubsystemBase {
    private final SparkMax topWheelMotor; // Algae intake
    private final SparkMax bottomWheelMotor; // Coral & algae outtake
    private final DigitalInput coralBreakSensor;

    /* Initializes motors. */
    public ClawWheelsSubsystem() {
        topWheelMotor = new SparkMax(ClawWheelsConstants.kTopWheelMotorID, MotorType.kBrushless);
        bottomWheelMotor = new SparkMax(ClawWheelsConstants.kBottomWheelMotorID, MotorType.kBrushless);
        coralBreakSensor = new DigitalInput(ClawWheelsConstants.kCoralBreakSensorDioPort);
    }

    /* Spins top wheel. */
    public void spinTopWheel(double speed) {
        topWheelMotor.set(speed);
    }

    public void spinBottomWheelVoltage(double voltage) {
        bottomWheelMotor.setVoltage(voltage);
    }

    public boolean checkCurrent() {
        return (bottomWheelMotor.getOutputCurrent() != 0.0);
    }

    public double getCurrent() {
        return bottomWheelMotor.getOutputCurrent();
    }

    /* Spins bottom wheel. */
    public void spinBottomWheel(double speed) {
        bottomWheelMotor.set(-speed);
    }
    
    /* Spins both wheels in sync, one is inverted for intake/outake instead of opposite directions. */
    public void spinWheelsSync(double speed) {
        topWheelMotor.set(speed);
        bottomWheelMotor.set(-speed);
    }

    /* Stops bottom wheel after finished with coral. */
    public void stopBottomWheel() {
        bottomWheelMotor.stopMotor();
    }

    /* Stops all motors. */
    public void stopAll() {
        topWheelMotor.stopMotor();
        bottomWheelMotor.stopMotor();
    }

    /* Checks if coral is inside intake */
    public boolean coralInIntake() {
        return !coralBreakSensor.get(); // True if not broken
    }

    /* Stops bottom wheel if coral is detected */
    public void manageBottomWheel() {
        if (!coralInIntake()) {
            stopBottomWheel();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CLAW VOLTAGE", bottomWheelMotor.getBusVoltage()); // 0.2 V
        SmartDashboard.putNumber("CLAW CURRENT", bottomWheelMotor.getOutputCurrent());
        SmartDashboard.putBoolean("BREAK SENSOR", coralInIntake());
    }

}