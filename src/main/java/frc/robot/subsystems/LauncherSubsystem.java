// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.signals.MotorTypeValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {

    private final TalonFX launcher = new TalonFX(18); // Velocity motor
    private final TalonFX angle = new TalonFX(19); 
    

    public LauncherSubsystem() {
       TalonFXConfiguration config = new TalonFXConfiguration();

    /* Feedback */
    config.Feedback.SensorToMechanismRatio = 1.0;

    /* Velocity PID */
    config.Slot0.kP = 0.3;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.11;

    /* Motion Magic */
    config.MotionMagic.MotionMagicAcceleration = 60.0;
    config.MotionMagic.MotionMagicJerk = 300.0;

    /* Current Limits (safe for Kraken) */
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = 40;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 20;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    launcher.getConfigurator().apply(config);
    angle.getConfigurator().apply(config);
    }

   
     public void setLauncherVelocity(double velocityRps) {
        MotionMagicVelocityVoltage velRequest = new MotionMagicVelocityVoltage(0);
        velRequest.withVelocity(velocityRps);
        launcher.setControl(velRequest);
    }

    public void setAnglePosition(double targetRotations) {
        MotionMagicVoltage posRequest = new MotionMagicVoltage(0);
        posRequest.withPosition(targetRotations);
        angle.setControl(posRequest);
    }

    

    public void stop() {
        launcher.stopMotor();
        angle.stopMotor();
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity().getValueAsDouble();
    }

    public double getAngleVelocity() {
        return angle.getVelocity().getValueAsDouble();
    }
}