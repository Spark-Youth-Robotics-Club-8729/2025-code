package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.IntakeClawConstants;

public class IntakeClawSubsystem {
    private final SparkMax m_clawMotor;
    private final RelativeEncoder m_encoder;
    private final PIDController pidController;

    public IntakeClawSubsystem(int CANID) {
        m_clawMotor = new SparkMax(CANID, MotorType.kBrushless);
        
        m_encoder = m_clawMotor.getEncoder();
      
        // Initialize PID controller
        pidController = new PIDController(IntakeClawConstants.kP, IntakeClawConstants.kI, IntakeClawConstants.kD);
        // pidController.setTolerance(1.0); // the tolerance is set in the SetClawAngle Command


        m_encoder.setPosition(0);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Rotate the motor at the desired speed
     *
     * @param speed The speed
     */
    public void rotate(double speed) {
        m_clawMotor.set(speed); 
    }

    public void setDesiredPosition(double desiredPosition) {
        double currentPosition = getPosition();
        double output = pidController.calculate(currentPosition, desiredPosition);
    
        // Apply the calculated output to the motor
        rotate(output);
    } 

    // Method to stop the motor
    public void stop() {
        rotate(0);
    }
    
    /** Zeroes all the encoders. */
    public void resetEncoders() {
        m_encoder.setPosition(0);
    }
}

