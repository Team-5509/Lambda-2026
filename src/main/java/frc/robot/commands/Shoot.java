// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ConveyorSubsystem m_conveyorSubsystem;
private final LauncherSubsystem m_launcherSubsystem;
private final KickerSubsystem m_kickerSubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(ConveyorSubsystem conveyorsubsystem, LauncherSubsystem launcherSubsystem, KickerSubsystem kickerSubsystem) {
    m_conveyorSubsystem = conveyorsubsystem;
    m_launcherSubsystem = launcherSubsystem;
    m_kickerSubsystem = kickerSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcherSubsystem.ExtendHoodMM();
    m_launcherSubsystem.RunLauncherMM();
    m_kickerSubsystem.RunKickerMM();
    m_conveyorSubsystem.RunConveyorMM();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        m_launcherSubsystem.StopLauncherMM();
    m_kickerSubsystem.StopKickerMM();
    m_conveyorSubsystem.StopConveyorMM();
    m_launcherSubsystem.RetractHoodMM();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}