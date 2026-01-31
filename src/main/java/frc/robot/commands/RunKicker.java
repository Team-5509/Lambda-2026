// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.KickerSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;



/** An example command that uses an example subsystem. */
public class RunKicker extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final KickerSubsystem m_kickerSubsystem;
  private double m_speed = .5;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public RunKicker(KickerSubsystem subsystem) {
    m_kickerSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  public void setSpeed(double speed) {
    m_speed = speed;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_kickerSubsystem.setKickerMoter(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        m_kickerSubsystem.setKickerMoter(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}