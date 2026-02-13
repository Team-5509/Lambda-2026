// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  /* ==================== Hardware IDs ==================== */
  private static final int KICKER_MOTOR_ID = Constants.KickerSubsystemConstants.kKickerMotorId;

  // Motion Magic
  private static final double MM_CRUISE_VEL = 2.0; // rot/s
  private static final double MM_ACCEL = 6.0; // rot/s^2
  private static final double MM_JERK = 60.0; // rot/s^3

  // Kicker Speed
  private double speed = 100.0;
  private double speedIncrement = 10.0;

  /* ==================== Hardware ==================== */
  private TalonFX kickerMotor = new TalonFX(KICKER_MOTOR_ID);

  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public KickerSubsystem() {
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    /* ---- Motion Magic ---- */
    config.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VEL;
    config.MotionMagic.MotionMagicAcceleration = MM_ACCEL;
    config.MotionMagic.MotionMagicJerk = MM_JERK;

    /* ---- PID ---- */
    config.Slot0.kP = 60.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 5.0;
    config.Slot0.kV = 0.0;

    /* ---- Motor ---- */
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    kickerMotor.getConfigurator().apply(config);
  }

  /**
   * Command that stops the kicker motor with magic motion (closed loop control).
   *
   * @return a command
   */
  public Command StopKickerMM() {
    return runOnce(() -> {
      kickerMotor.setControl(
          motionMagic.withVelocity(0)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the kicker motor with magic motion (closed loop control) at
   * speed from constants.
   *
   * @return a command
   */
  public Command RunKickerMM() {
    return runOnce(() -> {
      kickerMotor.setControl(
          motionMagic.withVelocity(speed)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the kicker motor with magic motion (closed loop control) at
   * Supplied speed.
   *
   * @return a command
   */
  public Command RunKickerMM(double velocityRPS) {
    return runOnce(() -> {
      kickerMotor.setControl(
          motionMagic.withVelocity(velocityRPS)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the kicker motor at a certain speed.
   *
   * @return a command
   */
  public Command RunKickerCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          kickerMotor.set(speed);
        });
  }

  /**
   * Command that stops the kicker motor at 0 speed.
   *
   * @return a command
   */
  public Command StopKickerCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          kickerMotor.set(0);
        });
  }

  /**
   * Command that stops the kicker motor at 0 speed.
   *
   * @return a command
   */
  public Command SetKickerSpeedCommand(DoubleSupplier setSpeed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          speed = setSpeed.getAsDouble();
        });
  }

  /**
   * Command that increments speed up by certain value.
   *
   * @return a command
   */
  public Command IncrementKickerSpeedUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          speed = speed + speedIncrement;
        });
  }

  /**
   * Command that increments speed down by certain value.
   *
   * @return a command
   */
  public Command IncrementKickerSpeedDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          speed = speed - speedIncrement;
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean KickerCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}