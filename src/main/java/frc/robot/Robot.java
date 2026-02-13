// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;                 // <-- correct class

import java.nio.file.Files;
import java.nio.file.Path;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", "Lambda-2026");

    if (RobotBase.isReal()) {
      // ---- REAL ROBOT ----
      Logger.addDataReceiver(new WPILOGWriter());  // writes to /U/logs when USB is present
      Logger.addDataReceiver(new NT4Publisher());  // live view in AdvantageScope
    } else {
      // ---- DESKTOP SIM ----

      Logger.addDataReceiver(new NT4Publisher());
      boolean wantReplay = Boolean.parseBoolean(
          System.getenv().getOrDefault("AK_REPLAY", "false"));
      String replayPath = System.getenv("AKIT_LOG_PATH"); // only use if explicitly provided

      if (wantReplay && replayPath != null && !replayPath.isBlank()
          && Files.exists(Path.of(replayPath))) {
        // --- Replay sim (deterministic, fast) ---
        setUseTiming(false);
        Logger.setReplaySource(new WPILOGReader(replayPath));
        Logger.addDataReceiver(new WPILOGWriter(replayPath.replace(".wpilog", "_sim.wpilog")));
      } else {
        // --- Live sim (no replay) ---
        Logger.addDataReceiver(new NT4Publisher());        // live to AdvantageScope
        Logger.addDataReceiver(new WPILOGWriter("build/ak-logs")); // local logs while simming
        // keep normal timing in live sim
      }
    }

    Logger.start(); // Start logging after receivers are added
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // m_robotContainer.visionFL.periodic();
    // m_robotContainer.visionFR.periodic();
    // m_robotContainer.visionRL.periodic();
    // m_robotContainer.visionRR.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    m_robotContainer.visionFL.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    m_robotContainer.visionFL.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    var debugFieldFL = m_robotContainer.visionFL.getSimDebugField();
    debugFieldFL.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);

    m_robotContainer.visionFR.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    m_robotContainer.visionFR.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    var debugFieldFR = m_robotContainer.visionFL.getSimDebugField();
    debugFieldFR.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);

    m_robotContainer.visionRL.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    m_robotContainer.visionRL.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    var debugFieldRL = m_robotContainer.visionFL.getSimDebugField();
    debugFieldRL.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);

    m_robotContainer.visionRR.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    m_robotContainer.visionRR.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);
    var debugFieldRR = m_robotContainer.visionFL.getSimDebugField();
    debugFieldRR.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);
  }
}
