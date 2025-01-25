package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
    private final SparkMax m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    // Encoders of the swerve drive
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    // PID Controller
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    // Offset of chassis to module
    private double m_chassisAngularOffset = 0;
    // What the swerve module wants to be
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

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

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSparkMax.getClosedLoopController();
        m_turningClosedLoopController = m_turningSparkMax.getClosedLoopController();

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle. (how a single motor should move to reach desired angle + speed)
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}