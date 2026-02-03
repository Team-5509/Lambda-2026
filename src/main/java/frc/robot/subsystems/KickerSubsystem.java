// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  private TalonFX kickerMoter = new TalonFX(16);


  /** Creates a new KickerSubsystem and applies Motion Magic configuration. */
  public KickerSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    /* Feedback */
    config.Feedback.SensorToMechanismRatio = 1.0;

    /* Position/Velocity PID (tune these values for your mechanism) */
    config.Slot0.kP = 0.5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.0;

    /* Motion Magic (example values) */
    config.MotionMagic.MotionMagicAcceleration = 100.0;
    config.MotionMagic.MotionMagicJerk = 300.0;

    kickerMoter.getConfigurator().apply(config);
  }

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


  public void setKickerMoter(double speed) {
    kickerMoter.set(speed);
  }

  /**
   * Move kicker to a target position (rotations) using Motion Magic.
   * @param targetRotations rotations of the feedback sensor (motor rotations)
   */
  public void setKickerPosition(double targetRotations) {
    MotionMagicVoltage posRequest = new MotionMagicVoltage(0);
    posRequest.withPosition(targetRotations);
    kickerMoter.setControl(posRequest);
  }

  /**
   * Convenience helper: move kicker to a target angle in degrees using Motion Magic position control.
   * Assumes 1 motor rotation == 360 degrees. If your mechanism has gearing, adjust accordingly.
   * @param degrees target degrees
   */
  public void setKickerPositionDegrees(double degrees) {
    double rotations = degrees / 360.0;
    setKickerPosition(rotations);
  }

  public double getKickerPosition() {
    return kickerMoter.getPosition().getValueAsDouble();
  }

  public double getKickerVelocity() {
    return kickerMoter.getVelocity().getValueAsDouble();
  }

  public void stopKickerMoter() {
    kickerMoter.set(0);

  }
}