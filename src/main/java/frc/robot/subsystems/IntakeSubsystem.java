// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeSubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMoter = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  private TalonFX deployIntakeMoter = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  



  /* ==================== Hardware IDs ==================== */
    private static final int Intake_MOTOR_ID = Constants.IntakeSubsystemConstants.kIntakeMotorId;
    private static final int Deploy_Intake_MOTOR_ID = Constants.IntakeSubsystemConstants.kIntakeMotorId;
  
  // Motion Magic
    private static final double MM_CRUISE_VEL = 2.0;   // rot/s
    private static final double MM_ACCEL      = 6.0;   // rot/s^2
    private static final double MM_JERK       = 60.0;  // rot/s^3

  // Intake Speed
  private double speed = 100.0;
  private double speedIncrement = 10.0;
    /* ==================== Hardware ==================== */
  private TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  
  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

  // Position control helper: we provide incremental commands to nudge the deploy position.
  private double deployPositionIncrement = 0.05; // rotations per increment

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
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

    
    intakeMotor.getConfigurator().apply(config);
    // Also configure the deploy motor for MotionMagic position control
    deployIntakeMoter.getConfigurator().apply(config);
  }
  
  /**
   * Command that runs the intake motor at a certain speed.
   *
   * @return a command
   */
  public Command RunIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          intakeMotor.set(speed);
        });
  }

  /**
   * Command that stops the intake motor at 0 speed.
   *
   * @return a command
   */
  public Command StopIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          intakeMotor.set(0);
        });
  }

  /**
   * Command that stops the intake motor at 0 speed.
   *
   * @return a command
   */
  public Command SetIntakeSpeedCommand(DoubleSupplier setSpeed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
         speed=setSpeed.getAsDouble();
        });
  }

    /**
   * Command that increments speed up by certain value.
   *
   * @return a command
   */
  public Command IncrementIntakeSpeedUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
         speed=speed+speedIncrement;
        });
  }

      /**
   * Command that increments speed down by certain value.
   *
   * @return a command
   */
  public Command IncrementIntakeSpeedDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
         speed=speed-speedIncrement;
        });
  }

  /**
   * Set the deploy motor to a target position (in rotations) using MotionMagic position control.
   */
  public void setDeployPosition(double targetRotations) {
    MotionMagicVoltage posRequest = new MotionMagicVoltage(0);
    posRequest.withPosition(targetRotations);
    deployIntakeMoter.setControl(posRequest);
  }

  /**
   * Command that moves the deploy intake to a specified target (single-shot).
   */
  public Command MoveDeployToPositionCommand(DoubleSupplier target) {
    return runOnce(() -> setDeployPosition(target.getAsDouble()));
  }

  /**
   * Command that moves the deploy intake up by a small increment (relative to current position).
   */
  public Command MoveDeployIncrementUpCommand() {
    return runOnce(() -> {
      double current = deployIntakeMoter.getPosition().getValue().in(Units.Rotations);
      setDeployPosition(current + deployPositionIncrement);
    });
  }

  /**
   * Command that moves the deploy intake down by a small increment (relative to current position).
   */
  public Command MoveDeployIncrementDownCommand() {
    return runOnce(() -> {
      double current = deployIntakeMoter.getPosition().getValue().in(Units.Rotations);
      setDeployPosition(current - deployPositionIncrement);
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean IntakeCondition() {
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
