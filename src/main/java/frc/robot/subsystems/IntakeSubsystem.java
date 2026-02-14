// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.IntakeSubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMoter = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  private TalonFX deployIntakeMoter = new TalonFX(IntakeSubsystemConstants.kDeployIntakeMotorId);
  



  /* ==================== Hardware IDs ==================== */
    private static final int Intake_MOTOR_ID = Constants.IntakeSubsystemConstants.kIntakeMotorId;
    private static final int Deploy_Intake_MOTOR_ID = Constants.IntakeSubsystemConstants.kDeployIntakeMotorId;
    private static final int INTAKE_CANCODER_ID = Constants.IntakeSubsystemConstants.kDeployIntakeEncoderId;
  // Motion Magic
    private static final double MM_CRUISE_VEL = 2.0;   // rot/s
    private static final double MM_ACCEL      = 6.0;   // rot/s^2
    private static final double MM_JERK       = 60.0;  // rot/s^3

  // Intake Speed
  private double speed = 100.0;
  private double speedIncrement = 10.0;

  private static final double MIN_INTAKE_ROT = -0.5;
    private static final double MAX_INTAKE_ROT = 0.5;
  

    /* ==================== Hardware ==================== */
  private TalonFX intakeMotor = new TalonFX(IntakeSubsystemConstants.kIntakeMotorId);
  // Absolute encoder (CANcoder) used for deploy position feedback
  private final CANcoder deployEncoder = new CANcoder(IntakeSubsystemConstants.kDeployIntakeEncoderId);
  
  private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVoltage motionMagicPosistion = new MotionMagicVoltage(0);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    configureIntakeMotor();
    configureDeployMotor();
    configureEncoder();
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
  
  //configures absolute encoder
 private void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // CANcoder always reports ±0.5 rotations (±180°) in Phoenix 6
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        deployEncoder.getConfigurator().apply(config);
    }

  //configures posistion controlled moter
private void configureDeployMotor() {
  // Configure TalonFX to use the CANcoder as its remote feedback device
  TalonFXConfiguration config = new TalonFXConfiguration();


  
  /* ---- Feedback ---- */
  config.Feedback.FeedbackRemoteSensorID = INTAKE_CANCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0;

  /* ---- Motion Magic ---- */
  config.MotionMagic.MotionMagicCruiseVelocity = MM_CRUISE_VEL;
  config.MotionMagic.MotionMagicAcceleration = MM_ACCEL;
  config.MotionMagic.MotionMagicJerk = MM_JERK;

  /* ---- PID ---- */
  config.Slot0.kP = 60.0;
  config.Slot0.kI = 0.0;
  config.Slot0.kD = 5.0;
  config.Slot0.kV = 0.0;

  /* ---- Soft Limits ---- */
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_INTAKE_ROT;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_INTAKE_ROT;

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
     * Command that sets the intake position with magic motion (closed loop
     * control).
     *
     * @return a command
     */
  public Command DeployIntakeMM(DoubleSupplier positionSupplier) {
    return runOnce(() -> {
      deployIntakeMoter.setControl(
          motionMagicPosistion.withPosition(positionSupplier.getAsDouble())
              .withSlot(0));
    });
  }

  /**
     * Command that sets the intake position with magic motion (closed loop
     * control).
     *
     * @return a command
     */
  public Command RetractIntakeMM(DoubleSupplier positionSupplier) {
    return runOnce(() -> {
      deployIntakeMoter.setControl(
          motionMagicPosistion.withPosition(positionSupplier.getAsDouble())
              .withSlot(0));
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
