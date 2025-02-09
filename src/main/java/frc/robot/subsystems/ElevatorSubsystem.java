package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.Units;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_rightKraken = new TalonFX(ElevatorConstants.kRightKrakenCanId);
    private final TalonFX m_leftKraken = new TalonFX(ElevatorConstants.kLeftKrakenCanId);

    private final CANcoder m_rightEncoder;
    private final CANcoder m_leftEncoder;

    private final PIDController pidController;

    private final DigitalInput m_topLimitSwitch;
    private final DigitalInput m_bottomLimitSwitch;

    public ElevatorSubsystem () {
        m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort);
        m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort);


        m_rightEncoder = new CANcoder(ElevatorConstants.kRightEncoderCanId);
        m_leftEncoder = new CANcoder(ElevatorConstants.kLeftEncoderCanId);
        
        // Initialize PID controller
        pidController = new PIDController(ElevatorConstants.kKrakenP, ElevatorConstants.kKrakenI, ElevatorConstants.kKrakenD);
        pidController.setTolerance(ElevatorConstants.kKrakenTolerance);

        m_rightEncoder.setPosition(0);
        m_leftEncoder.setPosition(0);
    }

    public boolean isAtTop() {
        return !m_topLimitSwitch.get();  // Sends signal when switch is activated (before inversion -> true when not pressed)
    }

    public boolean isAtBottom() {
        return !m_bottomLimitSwitch.get();  
    }

    /**
     * Rotate the motor at the desired speed
     *
     * @param speed The speed 
     */
    public void rotateMotors(double speed) {
        if (speed > 0 && isAtTop()) {
            // Stop if moving up and top limit reached
            speed = 0;
        } else if (speed < 0 && isAtBottom()) {
            // Stop if moving down and bottom limit reached
            speed = 0;
        }
        m_rightKraken.set(speed); 
        m_leftKraken.set(speed);
    }

    public void setDesiredPosition(double desiredPosition) {
        double currentPosition = getPosition();
        double feedforward = ElevatorConstants.kGravityFeedForward;
        double output = pidController.calculate(currentPosition, desiredPosition) + feedforward;

        // elevator output
        output = Math.max(-1.0, Math.min(1.0, output));
    
        // Apply the calculated output to the motor
        rotateMotors(output);
    } 

    public double getPosition() {    
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

    // Checks current to determine if at bottom
    public boolean atBottom() {
        double currentRight = m_rightKraken.getStatorCurrent().getValueAsDouble();
        double currentLeft = m_leftKraken.getStatorCurrent().getValueAsDouble();

        if (currentRight > ElevatorConstants.kBottomCurrentThreshold || currentLeft > ElevatorConstants.kBottomCurrentThreshold) {
            return true;
        }
        return false;
    }

    public void resetEncodersAtBottom() {
        if (atBottom()) {
            resetEncoders();
        }
    }

    @Override
    public void periodic() {
        resetEncodersAtBottom();
    }

}
