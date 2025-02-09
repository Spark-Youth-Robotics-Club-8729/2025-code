package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.RotateClawConstants;

public class RotateClawSubsystem extends SubsystemBase{
    private final SparkMax m_clawMotor = new SparkMax(RotateClawConstants.kClawRotationCanId, MotorType.kBrushless);
    private final DutyCycleEncoder m_encoder; // Absolute encoder
    private final PIDController pidController;

    public RotateClawSubsystem() {
        m_encoder = new DutyCycleEncoder(RotateClawConstants.kClawEncoderDioPort);
      
        // Initialize PID controller
        pidController = new PIDController(RotateClawConstants.kP, RotateClawConstants.kI, RotateClawConstants.kD);
        pidController.setTolerance(1.0); // the tolerance is set in the SetClawAngle Command
    }

    /**
     * Returns the current position of the claw in rotations.
     *
     * @return The absolute position of the claw (in rotations)
     */
    public double getPosition() {
        return m_encoder.get()*360.0; // returns value between 0 and 1, turn into angle
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
    
    // PID controller is within tolerance
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }
}

