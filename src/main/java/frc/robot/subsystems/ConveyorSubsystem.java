// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
  /** Creates a new ConveyorSubsystem and applies Motion Magic configuration. */
  public ConveyorSubsystem() {
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

    conveyorMotor.getConfigurator().apply(config);
  }
  private TalonFX conveyorMotor = new TalonFX(15);
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ConveyorMethodCommand() {
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
  public boolean ConveyorCondition() {
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
  public void SetConveyorMotor(double speed){
    conveyorMotor.set(speed);
  }
public void stopconveyormotor(){
conveyorMotor.set(0);
}

    /**
     * Move conveyor to a target position (rotations) using Motion Magic.
     * @param targetRotations motor rotations
     */
    public void setConveyorPosition(double targetRotations) {
      MotionMagicVoltage posRequest = new MotionMagicVoltage(0);
      posRequest.withPosition(targetRotations);
      conveyorMotor.setControl(posRequest);
    }

    /**
     * Helper: move conveyor to target angle in degrees using Motion Magic position control.
     * Assumes 1 motor rotation == 360 degrees. Adjust if gearing differs.
     * @param degrees target degrees
     */
    public void setConveyorPositionDegrees(double degrees) {
      double rotations = degrees / 360.0;
      setConveyorPosition(rotations);
    }

    public double getConveyorPosition() {
      return conveyorMotor.getPosition().getValueAsDouble();
    }

    public double getConveyorVelocity() {
      return conveyorMotor.getVelocity().getValueAsDouble();
    }

}