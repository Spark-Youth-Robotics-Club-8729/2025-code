package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    // Declare motor controller for the climbing mechanism
    private SparkMax m_climbMotor = new SparkMax(ClimbConstants.kClimbSpinMotorCanId, MotorType.kBrushless);
    
    // Declare encoder to track motor position
    private DutyCycleEncoder m_climbEncoder;

    // Declare PID controller
    private final PIDController pidController;

    // Constructor: Initializes motor and encoder
    public ClimbSubsystem() {
        // Get encoder from the motor to track rotation position
        // m_climbEncoder = new DutyCycleEncoder(ClimbConstants.kClimbEncoderDioPort);

        // Initialize PID controller
        pidController = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);
        pidController.setTolerance(ClimbConstants.kTolerance); 
    }
    
    /**
     * Returns the current position of the climber in rotations.
     *
     * @return The absolute position of the climber (in rotations)
     */
    public double getPosition() {
        return m_climbEncoder.get();
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

    // PID controller is within tolerance
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }
}