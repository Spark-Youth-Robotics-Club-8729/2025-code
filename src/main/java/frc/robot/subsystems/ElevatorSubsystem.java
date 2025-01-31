package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
 import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;


/* need to change to CANrange encoder!! */
public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_rightKraken = new TalonFX(ElevatorConstants.kRightKrakenCanId);
    private final TalonFX m_leftKraken = new TalonFX(ElevatorConstants.kLeftKrakenCanId);

    private final RelativeEncoder m_rightEncoder;
    private final RelativeEncoder m_leftEncoder;
    
    private final PIDController pidController;

    public ElevatorSubsystem () {

        m_rightEncoder = m_rightKraken.getEncoder();
        m_leftEncoder = m_leftKraken.getEncoder();
        
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
        m_rightKraken.setRotate(speed); 
        m_leftKraken.setRotate(speed);
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
        return (m_rightEncoder.getPosition() + m_leftEncoder.getPosition()) / 2.0;
    }

    // Method to stop the motor
    public void stop() {
        rotate(0);
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
