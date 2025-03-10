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

// run pid until reaches -0.5

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
        pidController.setSetpoint(ElevatorConstants.kL4);


        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

        m_limitSwitch = new DigitalInput(ElevatorConstants.kLimitSwitchPort);
        resetEncoders();
    }

    // public boolean isAtTop() {
    //     return getPosition() >= ElevatorConstants.kL4;
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
        if (isAtMaxHeight(speed)) {
            holdPosition();
            return;
        }
        m_rightKraken.set(speed); 
        m_leftKraken.set(-speed);
    }

    public void setVoltage(double voltage) {
        if (voltage > 3.0) {
            voltage = 3.0;
        }

        if (voltage < -3.0) {
            voltage = -3.0;
        }

        m_rightKraken.setVoltage(voltage); 
        m_leftKraken.setVoltage(voltage);
    }

    // public double stallSpeed() {
    //     return feedforward.calculate(getVelocity())/ElevatorConstants.kV;
    // }

    public double setDesiredPosition(double desiredPosition) {

        double currentPosition = getPosition();

        //double feedforward = ElevatorConstants.kGravityFeedForward;
        double output = pidController.calculate(currentPosition, desiredPosition);

        double ffOutput = feedforward.calculate(0);

        double outputSpeed = output + ffOutput;

        // elevator output
        //outputVoltage = Math.max(-0.1, Math.min(0.1, outputVoltage));
    
        // Apply the calculated output to the motor
        return outputSpeed;
    } 

    public boolean checkDownwardsVoltage(double voltage) {
        if (voltage >= -0.5 && voltage < 0.0) {
            return true;
        }
        return false;
    }

    public void setSetpoint(double desiredPosition) {
        pidController.setSetpoint(desiredPosition);
    }
    // public double getVelocity() {
    //     double rightVelocity = m_rightKraken.getVelocity().getValue().in(Units.RotationsPerSecond);
    //     double leftVelocity = m_leftKraken.getVelocity().getValue().in(Units.RotationsPerSecond);
        
    //     return (rightVelocity + leftVelocity) / 2.0;
    // }

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
    public boolean isAtSetpoint(double speed, double desiredPosition, double currentPosition) {
        return (Math.abs(desiredPosition-currentPosition) < ElevatorConstants.kKrakenTolerance) || isAtMaxHeight(speed);
    }

    // Checks current to determine if at bottom
    public boolean atBottom() {
        double currentRight = m_rightKraken.getStatorCurrent().getValueAsDouble();
        double currentLeft = m_leftKraken.getStatorCurrent().getValueAsDouble();

        if (currentRight > ElevatorConstants.kBottomCurrentThreshold || currentLeft > ElevatorConstants.kBottomCurrentThreshold) {
            resetEncoders();
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
            // resetEncoders();
        }
    }

    public void holdPosition() {
        double ffOutput = feedforward.calculate(0);
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

    public boolean isAtMaxHeight(double speed) {
        if (speed > 0) {
            return !m_limitSwitch.get();
        }
        else {
            return false;
        }
    }

    public void resetPID() {
        pidController.reset();
    }

    @Override
    public void periodic() {
        // resetEncodersAtBottom();
        SmartDashboard.putNumber("ELEVATOR Current position ", getPosition());
        SmartDashboard.putNumber("Calculated voltage top", setDesiredPosition(ElevatorConstants.kL4));
        SmartDashboard.putNumber("Calculated speed bottom", setDesiredPosition(ElevatorConstants.kBottomPosition));
        SmartDashboard.putBoolean("Elevator at Max Height", isAtSetpoint(0.5, ElevatorConstants.kL4, getPosition()));
        SmartDashboard.putNumber("Elevator Setpoint", pidController.getSetpoint());
        SmartDashboard.putNumber("Current", m_rightKraken.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Current", m_leftKraken.getStatorCurrent().getValueAsDouble());

    }

}