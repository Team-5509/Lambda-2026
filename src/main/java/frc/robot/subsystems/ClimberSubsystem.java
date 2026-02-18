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
import frc.robot.Constants.Constants.ClimberSubsystemConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
 /* ==================== Hardware IDs ==================== */
  private static final int CLIMBER_MOTOR_ID = Constants.ClimberSubsystemConstants.kClimberMotorId;

  private static final double MM_CRUISE_VEL = 2.0; // rot/s
  private static final double MM_ACCEL = 6.0; // rot/s^2
  private static final double MM_JERK = 60.0; // rot/s^3

  /* ==================== Hardware ==================== */
  private TalonFX climberMotor = new TalonFX(CLIMBER_MOTOR_ID);

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withEnableFOC(true);

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    configureMotor();
  }

  private void configureMotor() {
  // Configure TalonFX to use the CANcoder as its remote feedback device
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
  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

  climberMotor.getConfigurator().apply(config);
  }

/**
   * Command raises the climber with magic motion (closed loop control)
   *
   * @return a command
   */
  public Command ExtendClimberMM(DoubleSupplier positionSupplier) {
    return runOnce(() -> {
      climberMotor.setControl(
          motionMagic.withPosition(positionSupplier.getAsDouble())
              .withSlot(0));
    });
  }

/**
   * Command lowers the climber with magic motion (closed loop control)
   *
   * @return a command
   */
  public Command LowerClimberMM(DoubleSupplier positionSupplier) {
    return runOnce(() -> {
      climberMotor.setControl(
          motionMagic.withPosition(positionSupplier.getAsDouble())
              .withSlot(0));
    });
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