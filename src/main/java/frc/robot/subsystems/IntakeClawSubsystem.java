package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.IntakeClawConstants;

public class IntakeClawSubsystem {
    private final SparkMax m_sparkMax;
    private final RelativeEncoder m_encoder;
    private final PIDController pidController;

    public IntakeClawSubsystem(int CANId) {
        m_sparkMax = new SparkMax(CANId, MotorType.kBrushless);
        SparkMaxConfig configDriving = new SparkMaxConfig();
        configDriving
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        m_sparkMax.configure(configDriving, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_sparkMax.getEncoder();
      
        // Initialize PID controller
        pidController = new PIDController(IntakeClawConstants.kP, IntakeClawConstants.kI, IntakeClawConstants.kD);
        pidController.setTolerance(1.0); // Set acceptable error tolerance


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
        m_sparkMax.set(speed); 
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

