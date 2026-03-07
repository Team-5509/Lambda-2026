// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class HomeTurret extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem m_subsystem;

  /**
   * Creates a new HomeTurret command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HomeTurret(TurretSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /**
   * Called every scheduler cycle while the command is active.
   * Slowly drives the turret toward the negative limit switch at low speed.
   */
  @Override
  public void execute() {
    m_subsystem.setSpeed(-0.1);
  }

  /**
   * Called once when the command ends or is interrupted.
   * Stops the turret motor.
   *
   * @param interrupted Whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  /**
   * Returns true when the turret has reached the negative (home) limit switch.
   *
   * @return true if the negative limit switch is triggered
   */
  @Override
  public boolean isFinished() {
    return m_subsystem.isAtNegativeLimit();
  }
}