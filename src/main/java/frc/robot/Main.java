// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  /**
   * Main entry point for the robot program.
   * Starts the WPILib robot framework using the Robot class as the implementation.
   *
   * @param args Command-line arguments (not used)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
