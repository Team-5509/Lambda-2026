// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.RunKicker;
import frc.robot.commands.RunTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.CameraManager;
import frc.robot.Constants.CameraManager.CameraProperties;

public class RobotContainer {
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

          
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use closed-loop velocity control for driving

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController auxXbox = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final Vision visionFL = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_FL);
  public final Vision visionFR = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_FR);
  public final Vision visionRL = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_RL);
  public final Vision visionRR = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_RR);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

   private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("PlsDontExplode");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {
        Command runTurret = new RunTurret(m_turretSubsystem, () -> auxXbox.getLeftX());
        Command runKicker = new RunKicker(m_kickerSubsystem);
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MathUtil.applyDeadband(driverXbox.getLeftY(), 0.05)
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MathUtil.applyDeadband(driverXbox.getLeftX(), 0.05)
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -MathUtil.applyDeadband(driverXbox.getRightX(), 0.05)
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
    driverXbox.y().onTrue((drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));
    driverXbox.x().whileTrue(drivetrain.applyRequest(() -> brake));
     driverXbox.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverXbox.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driverXbox.povRight().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        driverXbox.povLeft().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );

    auxXbox.axisMagnitudeGreaterThan(1, 0.2).whileTrue(runTurret);
    auxXbox.a().whileTrue(runKicker );
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode) is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
  
   
}
