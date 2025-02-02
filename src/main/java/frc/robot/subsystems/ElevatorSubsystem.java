package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.units.Units;

/* need to change to CANrange encoder!! */
public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_rightKraken = new TalonFX(ElevatorConstants.kRightKrakenCanId);
    private final TalonFX m_leftKraken = new TalonFX(ElevatorConstants.kLeftKrakenCanId);

    private final CANcoder m_rightEncoder;
    private final CANcoder m_leftEncoder;

    
    private final PIDController pidController;

    public ElevatorSubsystem () {

        m_rightEncoder = new CANcoder(ElevatorConstants.kRightEncoderCanId);
        m_leftEncoder = new CANcoder(ElevatorConstants.kLeftEncoderCanId);
        
        // Initialize PID controller
        pidController = new PIDController(ElevatorConstants.kKrakenP, ElevatorConstants.kKrakenI, ElevatorConstants.kKrakenD);
        pidController.setTolerance(0.8);

        m_rightEncoder.setPosition(0);
        m_leftEncoder.setPosition(0);
    }

    /**
     * Rotate the motor at the desired speed
     *
     * @param speed The speed 
     */
    public void rotateMotors(double speed) {
        m_rightKraken.set(speed); 
        m_leftKraken.set(speed);
    }

    public void setDesiredPosition(double desiredPosition) {
        double currentPosition = getPosition();
        double output = pidController.calculate(currentPosition, desiredPosition);

        // elevator output
        output = Math.max(-1.0, Math.min(1.0, output));
    
        // Apply the calculated output to the motor
        rotateMotors(output);
    } 

    public double getPosition() {
        // averaging position because they should be in sync
        m_rightEncoder.getPosition().refresh();
        m_leftEncoder.getPosition().refresh();
    
        // Convert to rotations
        double rightPosition = m_rightEncoder.getPosition().getValue().in(Units.Rotations);
        double leftPosition = m_leftEncoder.getPosition().getValue().in(Units.Rotations);
    
        return (rightPosition + leftPosition) / 2.0;
    }

    // Method to stop the motor
    public void stop() {
        rotateMotors(0);
    }
    
    // Zeroes all the encoders.
    public void resetEncoders() {
        m_rightEncoder.setPosition(0);
        m_leftEncoder.setPosition(0);
    }

    // PID controller is within tolerance
    public boolean isAtSetpoint() {
        return pidController.atSetpoint();
    }

}
