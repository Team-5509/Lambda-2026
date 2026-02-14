// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.Constants.LauncherSubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  /* ==================== Hardware IDs ==================== */
  private static final int LAUNCHER_MOTOR_ID = Constants.LauncherSubsystemConstants.kLauncherMotorId;
   private static final int HOOD_MOTOR_ID = Constants.LauncherSubsystemConstants.kHoodMotorId;
    private static final int HOOD_CANCODER_ID = Constants.LauncherSubsystemConstants.kHoodEncoderId;


  private static final double MM_CRUISE_VEL = 2.0; // rot/s
  private static final double MM_ACCEL = 6.0; // rot/s^2
  private static final double MM_JERK = 60.0; // rot/s^3
 
  private static final double MM_CRUISE_VEL_HOOD = 2.0; // rot/s
  private static final double MM_ACCEL_HOOD = 6.0; // rot/s^2
  private static final double MM_JERK_HOOD = 60.0; // rot/s^3

  // Launcher Speed
  private double speed = 100.0;
  private double speedIncrement = 10.0;

   private static final double MIN_HOOD_ROT = -0.5;
   private static final double MAX_HOOD_ROT = 0.5;

  private double retractPosistion = 0;
  private double extendPosistion = 0.5;
  private double posistionIncrement = .05;

  /* ==================== Hardware ==================== */
  private TalonFX launcherMotor = new TalonFX(LAUNCHER_MOTOR_ID);
  private TalonFX hoodMotor = new TalonFX(HOOD_MOTOR_ID);

  private final CANcoder hoodEncoder = new CANcoder(LauncherSubsystemConstants.kHoodMotorId);

  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVoltage motionMagicHood = new MotionMagicVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public LauncherSubsystem() {
    configureEncoder();
    configureHoodMotor();
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

    launcherMotor.getConfigurator().apply(config);
  }

 //configures absolute encoder
 private void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // CANcoder always reports ±0.5 rotations (±180°) in Phoenix 6
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        hoodEncoder.getConfigurator().apply(config);
    }

   //configures posistion controlled moter
private void configureHoodMotor() {
  // Configure TalonFX to use the CANcoder as its remote feedback device
  TalonFXConfiguration config = new TalonFXConfiguration();


  
  /* ---- Feedback ---- */
  config.Feedback.FeedbackRemoteSensorID = HOOD_CANCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0; //TODO set ratio

  /* ---- Motion Magic ---- */
  config.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VEL_HOOD;
  config.MotionMagic.MotionMagicAcceleration = MM_ACCEL_HOOD;
  config.MotionMagic.MotionMagicJerk = MM_JERK_HOOD;

  /* ---- PID ---- */
  config.Slot0.kP = 60.0;
  config.Slot0.kI = 0.0;
  config.Slot0.kD = 5.0;
  config.Slot0.kV = 0.0;

  /* ---- Soft Limits ---- */
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_HOOD_ROT;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_HOOD_ROT;

  /* ---- Motor ---- */
  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

  hoodMotor.getConfigurator().apply(config);
  }

  /**
   * Command that stops the launcher motor with magic motion (closed loop control).
   *
   * @return a command
   */
  public Command StopLauncherMM() {
    return runOnce(() -> {
      launcherMotor.setControl(
          motionMagic.withVelocity(0)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the launcher motor with magic motion (closed loop control) at
   * speed from constants.
   *
   * @return a command
   */
  public Command RunLauncherMM() {
    return runOnce(() -> {
      launcherMotor.setControl(
          motionMagic.withVelocity(speed)
              .withSlot(0));
    });
  }

  /**
   * Command that runs the launcher motor with magic motion (closed loop control) at
   * Supplied speed.
   *
   * @return a command
   */
  public Command RunLauncherMM(DoubleSupplier velocityRPS) {
    return runOnce(() -> {
      launcherMotor.setControl(
          motionMagic.withVelocity(velocityRPS.getAsDouble())
              .withSlot(0));
    });
  }

    /**
   * Command Retracts the hood with magic motion (closed loop control)
   *
   * @return a command
   */
  public Command RetractHoodMM() {
    return runOnce(() -> {
      hoodMotor.setControl(
          motionMagicHood.withPosition(retractPosistion)
              .withSlot(0));
    });
  }

   /**
   * Command Extends the hood with magic motion (closed loop control)
   *
   * @return a command
   */
  public Command ExtendHoodMM() {
    return runOnce(() -> {
      hoodMotor.setControl(
          motionMagicHood.withPosition(extendPosistion)
              .withSlot(0));
    });
  }

  /**
   * Command that moves posistion down by certain value.
   *
   * @return a command
   */
  public Command MoveHoodDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          retractPosistion = retractPosistion - posistionIncrement;
        });
  }

  /**
   * Command that moves posistion down by certain value.
   *
   * @return a command
   */
  public Command MoveHoodUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          extendPosistion = extendPosistion + posistionIncrement;
        });
  }

/**
   * Command extends the hood with magic motion (closed loop control)
   *
   * @return a command
   */
  public Command ExtendHoodMM(DoubleSupplier posistionSupplier) {
    return runOnce(() -> {
      hoodMotor.setControl(
          motionMagicHood.withPosition(posistionSupplier.getAsDouble())
              .withSlot(0));
    });
  }

  /**
   * Command that runs the launcher motor at a certain speed.
   *
   * @return a command
   */
  public Command RunLauncherCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          launcherMotor.set(speed);
        });
  }

  /**
   * Command that stops the launcher motor at 0 speed.
   *
   * @return a command
   */
  public Command StopLauncherCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          launcherMotor.set(0);
        });
  }

  /**
   * Command that stops the launcher motor at 0 speed.
   *
   * @return a command
   */
  public Command SetLauncherSpeedCommand(DoubleSupplier setSpeed) {
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
  public Command IncrementLauncherSpeedUp() {
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
  public Command IncrementLauncherSpeedDown() {
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
  public boolean LauncherCondition() {
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