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

  /**
   * Robot constructor. Initializes the RobotContainer and configures AdvantageKit logging.
   * On a real robot, writes logs to USB and publishes live to AdvantageScope via NT4.
   * In simulation, supports optional log replay or live simulation with local log output.
   */
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

  /**
   * Runs every scheduler cycle (~20ms) regardless of robot mode.
   * Runs the WPILib CommandScheduler to execute active commands and poll triggers.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // m_robotContainer.visionFL.periodic();
    // m_robotContainer.visionFR.periodic();
    // m_robotContainer.visionRL.periodic();
    // m_robotContainer.visionRR.periodic();
  }

  /** Called once when the robot enters the disabled state. */
  @Override
  public void disabledInit() {}

  /** Called periodically while the robot is disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Called once when the robot exits the disabled state. */
  @Override
  public void disabledExit() {}

  /**
   * Called once when autonomous mode begins.
   * Retrieves and schedules the autonomous command selected on the SmartDashboard.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** Called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once when autonomous mode ends. */
  @Override
  public void autonomousExit() {}

  /**
   * Called once when teleoperated mode begins.
   * Cancels any autonomous command that may still be running.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {}

  /** Called once when teleoperated mode ends. */
  @Override
  public void teleopExit() {}

  /**
   * Called once when test mode begins.
   * Cancels all currently scheduled commands to start with a clean state.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when test mode ends. */
  @Override
  public void testExit() {}

  /**
   * Called periodically during simulation.
   * Updates all four vision camera simulations with the current robot pose
   * and sets the estimated robot pose on each debug field visualization.
   */
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
