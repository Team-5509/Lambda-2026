// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.KickerSubsystem;

/**
 * Fires the ball by running the kicker and conveyor.
 *
 * <p>Intended to run while a trigger button is held. Because {@link ShootingArcCommand}
 * keeps the launcher continuously spun up and the hood pre-positioned, no spin-up
 * delay is needed — the shot fires immediately.
 *
 * <p>This command only requires {@link KickerSubsystem} and {@link ConveyorSubsystem}
 * so it does not interrupt the always-running {@link ShootingArcCommand}.
 */
public class FireCommand extends Command {

  private final KickerSubsystem   m_kickerSubsystem;
  private final ConveyorSubsystem m_conveyorSubsystem;

  /**
   * @param kickerSubsystem    Kicker subsystem
   * @param conveyorSubsystem  Conveyor subsystem
   */
  public FireCommand(KickerSubsystem kickerSubsystem, ConveyorSubsystem conveyorSubsystem) {
    m_kickerSubsystem   = kickerSubsystem;
    m_conveyorSubsystem = conveyorSubsystem;

    addRequirements(kickerSubsystem, conveyorSubsystem);
  }

  @Override
  public void execute() {
    m_kickerSubsystem.RunKickerMM();
    m_conveyorSubsystem.RunConveyorMM();
  }

  @Override
  public void end(boolean interrupted) {
    m_kickerSubsystem.StopKickerMM();
    m_conveyorSubsystem.StopConveyorMM();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
