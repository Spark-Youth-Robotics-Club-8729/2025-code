package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.RotateClawConstants;

public class RotateClawSubsystem extends SubsystemBase{
    private final SparkMax m_clawMotor = new SparkMax(RotateClawConstants.kClawRotationCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder;
    private final PIDController pidController;

    public RotateClawSubsystem() {
        m_encoder = m_clawMotor.getEncoder();
      
        // Initialize PID controller
        pidController = new PIDController(RotateClawConstants.kP, RotateClawConstants.kI, RotateClawConstants.kD);
        pidController.setTolerance(1.0); // the tolerance is set in the SetClawAngle Command


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
        m_encoder.setPosition(0);
    }

    // PID controller is within tolerance
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }
}

