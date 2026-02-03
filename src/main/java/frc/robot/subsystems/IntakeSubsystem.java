// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem and applies Motion Magic configuration. */
  public IntakeSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    /* Feedback */
    config.Feedback.SensorToMechanismRatio = 1.0;

    /* Position/Velocity PID (tune these values for your mechanism) */
    config.Slot0.kP = 0.4;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.0;

    /* Motion Magic (example values) */
    config.MotionMagic.MotionMagicAcceleration = 80.0;
    config.MotionMagic.MotionMagicJerk = 300.0;

    intakeMotor.getConfigurator().apply(config);
  }
    private TalonFX intakeMotor = new TalonFX(4);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
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
  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }
  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  /**
   * Move intake to target position (rotations) using Motion Magic.
   * @param targetRotations motor rotations
   */
  public void setIntakePosition(double targetRotations) {
    MotionMagicVoltage posRequest = new MotionMagicVoltage(0);
    posRequest.withPosition(targetRotations);
    intakeMotor.setControl(posRequest);
  }

  /**
   * Helper: move intake to target angle in degrees using Motion Magic position control.
   * Assumes 1 motor rotation == 360 degrees. Adjust if gearing differs.
   * @param degrees target degrees
   */
  public void setIntakePositionDegrees(double degrees) {
    double rotations = degrees / 360.0;
    setIntakePosition(rotations);
  }

  public double getIntakePosition() {
    return intakeMotor.getPosition().getValueAsDouble();
  }

  public double getIntakeVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }
}
