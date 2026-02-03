// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ConveyorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem m_subsystem;
  private final Double m_speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConveyorCommand(ConveyorSubsystem subsystem, double speed ) {
    m_subsystem = subsystem;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Use Motion Magic position control from ConveyorSubsystem.
    // Interpret m_speed as a fraction of one rotation (0..1). Convert to degrees.
    double targetDegrees = m_speed * 360.0;
    m_subsystem.setConveyorPositionDegrees(targetDegrees);
  }
   
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
m_subsystem.SetConveyorMotor(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}