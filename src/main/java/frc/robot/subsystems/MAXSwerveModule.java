// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // When the wheel speed command is ~0, continuously holding steering angle with
  // position PID can cause the module to "fight" during rapid direction changes.
  // This threshold is in m/s (because we command driving velocity in m/s).
  private static final double kSteerHoldMinSpeedMetersPerSecond = 0.05;

  // Soft-hold: remember the last commanded steering angle (radians, corrected).
  // When we're basically stopped and holdSteeringAngle=false, we freeze this
  // setpoint instead of continually updating it (prevents twitch/flip).
  private double m_lastSteerSetpointRad = 0.0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_lastSteerSetpointRad = m_turningEncoder.getPosition() + m_chassisAngularOffset;
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the raw absolute encoder position from the turning encoder (radians),
   * without applying the software inversion flag.
   */
  public double getRawTurningEncoderPosition() {
    return m_turningEncoder.getPosition();
  }

  /**
   * Returns the effective turning encoder position used by the module code
   * (radians), i.e. after applying the software inversion flag if enabled.
   */
  public double getEffectiveTurningEncoderPosition() {
    // Currently no software inversion flag is applied here; return the encoder value.
    return m_turningEncoder.getPosition();
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
    setDesiredState(desiredState, true);
  }

  /**
   * Sets the desired state for the module with optional steering hold.
   *
   * @param desiredState Desired state with speed and angle.
   * @param holdSteeringAngle If true, steer motor runs closed-loop position to hold
   *                          the requested angle. If false and speed is near zero,
   *                          the steer motor output is set to 0 to avoid fighting.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean holdSteeringAngle) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving SPARK towards the velocity setpoint.
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

    // Steering control:
    // - AutoAim (holdSteeringAngle=true): track the requested angle aggressively.
    // - Normal teleop (holdSteeringAngle=false): if we're basically stopped, freeze
    //   the last steering setpoint to avoid flip/jitter from rapid command changes.
    double requestedSteerRad;
    if (holdSteeringAngle || Math.abs(correctedDesiredState.speedMetersPerSecond) > kSteerHoldMinSpeedMetersPerSecond) {
      requestedSteerRad = correctedDesiredState.angle.getRadians();
      m_lastSteerSetpointRad = requestedSteerRad;
    } else {
      requestedSteerRad = m_lastSteerSetpointRad;
    }
    m_turningClosedLoopController.setSetpoint(requestedSteerRad, ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
