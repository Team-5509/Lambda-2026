// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunTurret extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem m_turretSubsystem;
  private final DoubleSupplier m_controllerInput;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunTurret(TurretSubsystem subsystem, DoubleSupplier controllerInput) {
    m_turretSubsystem = subsystem;
    m_controllerInput = controllerInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Map controller axis (-1..1) to turret angle in degrees.
    // Change maxAngleDegrees to the safe range for your turret (e.g. 180 for +/-180°).
    double maxAngleDegrees = 180.0;
    double targetDegrees = m_controllerInput.getAsDouble() * maxAngleDegrees;
    m_turretSubsystem.setTurretPositionDegrees(targetDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretSubsystem.setTurretMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}