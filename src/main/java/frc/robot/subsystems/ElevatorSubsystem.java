package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.controller.ElevatorFeedforward;  

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX m_rightKraken = new TalonFX(ElevatorConstants.kRightKrakenCanId);
    private final TalonFX m_leftKraken = new TalonFX(ElevatorConstants.kLeftKrakenCanId);

    private final PIDController pidController;

    private final ElevatorFeedforward feedforward;
    private final DigitalInput m_limitSwitch;

    public ElevatorSubsystem () {
        // Initialize PID controller
        pidController = new PIDController(ElevatorConstants.kKrakenP, ElevatorConstants.kKrakenI, ElevatorConstants.kKrakenD);
        pidController.setTolerance(ElevatorConstants.kKrakenTolerance);

        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

        m_limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
        resetEncoders();
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
        if (isAtMaxHeight() && speed > 0) {
            stop();
            return;
        }

        m_rightKraken.set(speed); 
        m_leftKraken.set(speed);
    }

    public void setVoltage(double voltage) {
        m_rightKraken.setVoltage(voltage); 
        m_leftKraken.setVoltage(voltage);
    }

    public double setDesiredPosition(double desiredPosition) {
        if (isAtMaxHeight() && desiredPosition > ElevatorConstants.kTopPosition) {
            stop();
            return 0;
        }
        
        double currentPosition = getPosition();
        //double feedforward = ElevatorConstants.kGravityFeedForward;
        double output = pidController.calculate(currentPosition, desiredPosition);

        double velocity = getVelocity();
        double ffOutput = feedforward.calculate(velocity)/ElevatorConstants.kV;

        double outputSpeed = output + ffOutput;

        // elevator output
        //outputVoltage = Math.max(-0.1, Math.min(0.1, outputVoltage));
    
        // Apply the calculated output to the motor
        return outputSpeed;
    } 

    public double getVelocity() {
        double rightVelocity = m_rightKraken.getVelocity().getValue().in(Units.RotationsPerSecond);
        double leftVelocity = m_leftKraken.getVelocity().getValue().in(Units.RotationsPerSecond);
        
        return (rightVelocity + leftVelocity) / 2.0;
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

    public void holdPosition(double desiredPosition) {
        double velocity = getVelocity();
        double ffOutput = feedforward.calculate(velocity);
        setVoltage(ffOutput);
    }

    public void setCoast() {
        m_leftKraken.setNeutralMode(NeutralModeValue.Coast);
        m_rightKraken.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBrake() {
        m_leftKraken.setNeutralMode(NeutralModeValue.Brake);
        m_rightKraken.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean isAtMaxHeight() {
        return !m_limitSwitch.get();
    }

    @Override
    public void periodic() {
        resetEncodersAtBottom();
        SmartDashboard.putNumber("ELEVATOR Current position ", getPosition());
        SmartDashboard.putNumber("Calculated speed top", setDesiredPosition(ElevatorConstants.kTopPosition));
        SmartDashboard.putNumber("Calculated speed bottom", setDesiredPosition(ElevatorConstants.kBottomPosition));
        SmartDashboard.putBoolean("Elevator at Max Height", isAtMaxHeight());
    }

}