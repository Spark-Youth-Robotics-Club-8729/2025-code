package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_rightKraken = new TalonFX(ElevatorConstants.kRightKrakenCanId);
    private final TalonFX m_leftKraken = new TalonFX(ElevatorConstants.kLeftKrakenCanId);

    private final PIDController pidController;

    public ElevatorSubsystem () {
        // Initialize PID controller
        pidController = new PIDController(ElevatorConstants.kKrakenP, ElevatorConstants.kKrakenI, ElevatorConstants.kKrakenD);
        pidController.setTolerance(ElevatorConstants.kKrakenTolerance);
    }

    // public boolean isAtTop() {
    //     return getPosition() >= ElevatorConstants.kTopPosition;
    //     // return !m_topLimitSwitch.get();  // Sends signal when switch is activated (before inversion -> true when not pressed)
    // }

    // public boolean isAtBottom() {
    //     return getPosition() <= ElevatorConstants.kBottomPosition;
    //     // return !m_bottomLimitSwitch.get();  
    // }

    /**
     * Rotate the motor at the desired speed
     *
     * @param speed The speed 
     */
    public void rotate(double speed) {
        m_rightKraken.set(speed); 
        m_leftKraken.set(-speed);
    }

    public double setDesiredPosition(double desiredPosition) {
        double currentPosition = getPosition();
        double feedforward = ElevatorConstants.kGravityFeedForward;
        double output = pidController.calculate(currentPosition, desiredPosition) + feedforward;

        // elevator output
        output = Math.max(-0.3, Math.min(0.3, output));
    
        // Apply the calculated output to the motor
        return output;
    } 

    public double getPosition() {    
        // Convert to rotations
        double rightPosition = m_rightKraken.getPosition().getValue().in(Units.Rotations);
        double leftPosition = m_leftKraken.getPosition().getValue().in(Units.Rotations);
    
        return (rightPosition + leftPosition) / 2.0;
    }

    // Method to stop the motor
    public void stop() {
        rotate(0);
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

    // Zeroes all the encoders.
    public void resetEncoders() {
        m_rightKraken.setPosition(0);
        m_leftKraken.setPosition(0);
    }

    public void resetEncodersAtBottom() {
        if (atBottom()) {
            resetEncoders();
        }
    }

    public void resetPID() {
        pidController.reset();
    }

    @Override
    public void periodic() {
        resetEncodersAtBottom();
        SmartDashboard.putNumber("Current position", getPosition());
        SmartDashboard.putNumber("Calculated speed top", setDesiredPosition(ElevatorConstants.kTopPosition));
        SmartDashboard.putNumber("Calculated speed bottom", setDesiredPosition(ElevatorConstants.kBottomPosition));
    }

}