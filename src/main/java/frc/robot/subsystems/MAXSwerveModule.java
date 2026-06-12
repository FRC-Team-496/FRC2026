// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkFlex m_turningSparkFlex;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private int drivingCANId;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this.drivingCANId = drivingCANId;
    m_drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkFlex = new SparkFlex(turningCANId, MotorType.kBrushless);
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    SparkFlexConfig drivingConfig = new SparkFlexConfig();

   
    drivingConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

drivingConfig.encoder
    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

drivingConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ModuleConstants.kDrivingP,
         ModuleConstants.kDrivingI,
         ModuleConstants.kDrivingD)
    .outputRange(ModuleConstants.kDrivingMinOutput,
                 ModuleConstants.kDrivingMaxOutput);

drivingConfig.closedLoop.feedForward
    .kV(ModuleConstants.kDrivingFF);

m_drivingSparkFlex.configure(
    drivingConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters
);

    SparkFlexConfig turningConfig = new SparkFlexConfig();

turningConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

turningConfig.absoluteEncoder
    .inverted(true)
    .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

turningConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .positionWrappingEnabled(true)
    .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
    .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
    .pid(ModuleConstants.kTurningP,
         ModuleConstants.kTurningI,
         ModuleConstants.kTurningD)
    .outputRange(ModuleConstants.kTurningMinOutput,
                 ModuleConstants.kTurningMaxOutput);

turningConfig.closedLoop.feedForward
    .kV(ModuleConstants.kTurningFF);   

    m_turningSparkFlex.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkFlex.getAbsoluteEncoder(); //removed "Type.kDutyCycle" from parameter


    m_drivingPIDController = m_drivingSparkFlex.getClosedLoopController();
    m_turningPIDController = m_turningSparkFlex.getClosedLoopController();
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
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

   SwerveModuleState optimizedDesiredState = new SwerveModuleState(
    correctedDesiredState.speedMetersPerSecond,
    correctedDesiredState.angle
);

optimizedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

// Command driving and turning SPARKS MAX/Flex towards their respective setpoints.
m_drivingPIDController.setSetpoint(
    optimizedDesiredState.speedMetersPerSecond,
    SparkBase.ControlType.kVelocity
);

m_turningPIDController.setSetpoint(
    optimizedDesiredState.angle.getRadians(),
    SparkBase.ControlType.kPosition
);
    if(drivingCANId == DriveConstants.kFrontLeftDrivingCanId){
      SmartDashboard.putNumber("FL input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Left cmd rot", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Left Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Front Left angle", m_turningEncoder.getPosition());

    }
    if(drivingCANId == DriveConstants.kFrontRightDrivingCanId){
      SmartDashboard.putNumber("FR input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Right Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Front Right Rotation", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Front Right angle", m_turningEncoder.getPosition());

    }
    if(drivingCANId == DriveConstants.kRearLeftDrivingCanId){
      SmartDashboard.putNumber("BL input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Left Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Back Left Rotation", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Left angle", m_turningEncoder.getPosition());

    }
    if(drivingCANId == DriveConstants.kRearRightDrivingCanId){
      SmartDashboard.putNumber("BR input Rot", desiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Right Speed", optimizedDesiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("Back Right Rotation", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("Back Right angle", m_turningEncoder.getPosition());

    }
    m_desiredState = desiredState;

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
