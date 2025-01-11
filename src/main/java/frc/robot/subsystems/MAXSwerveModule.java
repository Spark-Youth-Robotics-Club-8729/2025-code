package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */

   // New Migrartion Code:
    // https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    SparkMaxConfig configDriving = new SparkMaxConfig();
    configDriving
      .inverted(false)
      .idleMode(IdleMode.kBrake);
      configDriving.encoder
        .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
      configDriving.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD);
    m_drivingSparkMax.configure(configDriving, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    SparkMaxConfig configTurning = new SparkMaxConfig();
    configTurning
      .inverted(ModuleConstants.kTurningEncoderInverted)
      .idleMode(IdleMode.kBrake);
      configTurning.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
      configTurning.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
    m_turningSparkMax.configure(configTurning, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /* NEED TO FIGURE OUT HOW TO ADD ALL OF THE FOLLOWING CODE TO UPDATED CODE 
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput);

        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
            ModuleConstants.kTurningMaxOutput);

        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    */


    m_chassisAngularOffset = chassisAngularOffset;
    // Rotation is absolute encoder
    m_desiredState.angle = new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition());
    // Driving is relative encoder
    m_drivingSparkMax.getEncoder().setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingSparkMax.getEncoder().getVelocity(),
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingSparkMax.getEncoder().getPosition(),
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingSparkMax.getClosedLoopController().setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningSparkMax.getClosedLoopController().setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingSparkMax.getEncoder().setPosition(0);
  }

}