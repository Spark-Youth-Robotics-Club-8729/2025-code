package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.RotateClawConstants;

public class RotateClawSubsystem extends SubsystemBase{
    private final SparkMax m_clawMotor = new SparkMax(RotateClawConstants.kClawRotationCanId, MotorType.kBrushless);
    private final AbsoluteEncoder m_encoder; // Absolute encoder
    private final PIDController pidController;

    public RotateClawSubsystem() {
        m_encoder = m_clawMotor.getAbsoluteEncoder();
      
        // Initialize PID controller
        pidController = new PIDController(RotateClawConstants.kP, RotateClawConstants.kI, RotateClawConstants.kD);
        pidController.setTolerance(0.01); // the tolerance is set in the SetClawAngle Command
        pidController.enableContinuousInput(-180, 180);
    }

    /**
     * Returns the current position of the claw in rotations.
     *
     * @return The absolute position of the claw (in rotations)
     */
    public double getPosition() {
        return m_encoder.getPosition(); // returns value between 0 and 1, turn into angle
    }

    /**
     * Rotate the motor at the desired speed
     *
     * @param speed The speed
     */
    public void rotate(double speed) {
        m_clawMotor.set(speed); 
    }

    public double setDesiredPosition(double desiredPosition) {
        double currentPosition = getPosition();
        double error = desiredPosition - currentPosition;
    
        // Normalize error to always take the shortest rotation direction
        if (error > 0.5) {
            error -= 1.0;  // If the difference is greater than half a rotation, go backward
        } else if (error < -0.5) {
            error += 1.0;  // If the difference is less than -half a rotation, go forward
        }
        

        double output = pidController.calculate(currentPosition, currentPosition+error);

        // Clamp output
        // output = -Math.max(-0.05, Math.min(0.05, output));
        if (output > 0.075) {
            output = 0.075;
        }

        if (output < -0.075) {
            output = -0.075;
        }

        // Apply the calculated output to the motor
        return -output;
    } 

    public void resetPID() {
        pidController.reset();
    }

    // Method to stop the motor
    public void stop() {
        rotate(0.01);
    }
    
    // PID controller is within tolerance
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current position", getPosition());
        SmartDashboard.putNumber("Calculated speed", setDesiredPosition(0));
    }
}