// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeSubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMoter = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  private TalonFX deployIntakeMoter = new TalonFX(IntakeSubsystemConstants.kDeployIntakeMotorId);
  



  /* ==================== Hardware IDs ==================== */
    private static final int Intake_MOTOR_ID = Constants.IntakeSubsystemConstants.kIntakeMotorId;
    private static final int Deploy_Intake_MOTOR_ID = Constants.IntakeSubsystemConstants.kDeployIntakeMotorId;
  
  // Motion Magic
    private static final double MM_CRUISE_VEL = 2.0;   // rot/s
    private static final double MM_ACCEL      = 6.0;   // rot/s^2
    private static final double MM_JERK       = 60.0;  // rot/s^3

  // Intake Speed
  private double speed = 100.0;
  private double speedIncrement = 10.0;
  // Intake positions (rotations)
  private double deployPosition = 0.0;
  private double retractPosition = 0.0;
  private double deployIncrement = 5;

    /* ==================== Hardware ==================== */
  private TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  
  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    configureIntakeMotor();
    configureDeployMotor();
  }

  //configures velocity controlled motor
  private void configureIntakeMotor() {
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
  }
  
  //configures posistion controlled moter
private void configureDeployMotor() {
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

    
    deployIntakeMoter.getConfigurator().apply(config);
  }
  
  
/**
   * Command that stops the intake motor with magic motion (closed loop control).
   *
   * @return a command
   */
  public Command StopIntakeMM() {
    return runOnce(() -> {
      intakeMotor.setControl(
          motionMagic.withVelocity(0)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the intake motor with magic motion (closed loop control) at
   * speed from constants.
   *
   * @return a command
   */
  public Command RunIntakeMM() {
    return runOnce(() -> {
      intakeMotor.setControl(
          motionMagic.withVelocity(speed)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the intake motor with magic motion (closed loop control) at
   * supplied speed.
   *
   * @return a command
   */
  public Command RunIntakeMM(DoubleSupplier velocityRPS) {
    return runOnce(() -> {
      intakeMotor.setControl(
          motionMagic.withVelocity(velocityRPS.getAsDouble())
              .withSlot(0));
    });
  }

  /**
   * Command that sets the deployIntake motor deployed posistion with magic motion 
   * (closed loop control) using posistion variable.
   *
   * @return a command
   */
  public Command DeployIntakeMM() {
    return runOnce(() -> {
      MotionMagicVoltage pos = new MotionMagicVoltage(0);
      pos.withPosition(deployPosition).withSlot(0);
      deployIntakeMoter.setControl(pos);
    });
  }

  /**
   * Command that sets the deployIntake motor retracted posistion with magic motion 
   * (closed loop control) using posistion variable.
   *
   * @return a command
   */
   public Command RetractIntakeMM() {
    return runOnce(() -> {
      MotionMagicVoltage pos = new MotionMagicVoltage(0);
      pos.withPosition(retractPosition).withSlot(0);
      deployIntakeMoter.setControl(pos);
    });
  }

  /**
   * Command that runs the deployIntake motor deployed posistion with magic motion 
   * (closed loop control) to supplied posistion.
   *
   * @return a command
   */
public Command DeployIntakeMM(DoubleSupplier position) {
    return runOnce(() -> {
      MotionMagicVoltage pos = new MotionMagicVoltage(0);
      pos.withPosition(position.getAsDouble()).withSlot(0);
      deployIntakeMoter.setControl(pos);
    });
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
   * Command that increments speed up by certain value.
   *
   * @return a command
   */
  public Command IncrementDeployPositionUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
         deployPosition = deployPosition + deployIncrement;
        });
  }
    /**
   * Command that increments speed up by certain value.
   *
   * @return a command
   */
  public Command IncrementDeployPositionDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
         deployPosition = deployPosition - deployIncrement;
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
