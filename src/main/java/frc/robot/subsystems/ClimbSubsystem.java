package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
    // Declare motor controller for the climbing mechanism
    private SparkMax m_climbMotor = new SparkMax(ClimbConstants.kClimbSpinMotorCanId, MotorType.kBrushless);
    
    // Declare encoder to track motor position
    private RelativeEncoder m_climbEncoder;

    // Declare PID controller
    private final PIDController pidController;

    // Paths for logging motor speed and encoder data
    private final String CLIMBER_SPEED_LOG_PATH = "/Climber/Speeds";
    private final String CLIMBER_ENCODER_LOG_PATH = "/Climber/Encoder";

    // Constructor: Initializes motor and encoder
    public ClimbSubsystem() {
        // Get encoder from the motor to track rotation position
        m_climbEncoder = m_climbMotor.getEncoder();

        // Initialize PID controller
        pidController = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);
        pidController.setTolerance(ClimbConstants.kTolerance); // the tolerance is set in the SetClawAngle Command


        m_climbEncoder.setPosition(0);
    }
    
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public double getPosition() {
        return m_climbEncoder.getPosition();
    }

    /**
     * Rotate the motor at the desired speed
     *
     * @param speed The speed
     */
    public void rotate(double speed) {
        m_climbMotor.set(speed); 
    }

    public void setDesiredAngle(double desiredAngle) {
        double currentPosition = getPosition();
        double output = pidController.calculate(currentPosition, desiredAngle);

        // Clamp output
        output = Math.max(-1.0, Math.min(1.0, output));
    
        // Apply the calculated output to the motor
        rotate(output);
    } 

    // Method to stop the motor
    public void stop() {
        rotate(0);
    }
    
    /** Zeroes all the encoders. */
    public void resetEncoders() {
        m_climbEncoder.setPosition(0);
    }

    // PID controller is within tolerance
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }
    
    // Logs motor speed and encoder position for debugging and performance tracking
    public void logOutputs(){
        Logger.recordOutput(getName() + CLIMBER_SPEED_LOG_PATH, m_climbMotor.get()); // Log motor speed
        Logger.recordOutput(getName() + CLIMBER_ENCODER_LOG_PATH, getPosition()); // Log encoder position
    }

    @Override
    public void periodic() {
        logOutputs(); // Automatically logs every cycle
    }
}