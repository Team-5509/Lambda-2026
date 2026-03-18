// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.CameraManager.CameraProperties;
import frc.robot.commands.TrackFieldPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.AutoLaunchLookup;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchLookup;
import frc.robot.Constants.Constants.TurretSubsystemConstants;
import frc.robot.subsystems.ConveyorSubsystem;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top //
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // CCW+, field-relative

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); // Use closed-loop velocity
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // control for driving

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController auxXbox = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem(() -> drivetrain.getState().Pose);
    private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

    private double FinesseSpeedMult = 0.5;
    private double FinesseAngularRateMult = 0.5;

    // public final Vision visionFL = new Vision(drivetrain::addVisionMeasurement,
    // CameraProperties.CAM_FL);
    // public final Vision visionFR = new Vision(drivetrain::addVisionMeasurement,
    // CameraProperties.CAM_FR);

    public final Vision visionRL = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_RL);
    public final Vision visionR = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_R);
    public final Vision visionRR = new Vision(drivetrain::addVisionMeasurement, CameraProperties.CAM_RR);

    private Command makeLaunchLookup() {
        return new LaunchLookup(
                m_conveyorSubsystem,
                m_launcherSubsystem,
                m_kickerSubsystem,
                () -> drivetrain.getState().Pose,
                this::getFieldRelativeVelocity);
    }

    private Command autoRunLaunchLookupWithTimeout() {
        return new AutoLaunchLookup(
                m_conveyorSubsystem,
                m_launcherSubsystem,
                m_kickerSubsystem,
                () -> drivetrain.getState().Pose,
                this::getFieldRelativeVelocity).withTimeout(5); // seconds
    }

    boolean m_toggleRobotCentric = false;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register named commands after subsystem fields have been initialized
        NamedCommands.registerCommand("RunLauncher", m_launcherSubsystem.RunLauncherMM());
        NamedCommands.registerCommand("StopLauncher", m_launcherSubsystem.StopLauncherMM());
        NamedCommands.registerCommand("RunIntake", m_intakeSubsystem.RunIntakeMM());
        NamedCommands.registerCommand("StopIntake", m_intakeSubsystem.StopIntakeMM());
        NamedCommands.registerCommand("DeployIntake", m_intakeSubsystem.DeployIntakeMM());
        NamedCommands.registerCommand("RetractIntake", m_intakeSubsystem.RetractIntakeMM());
        NamedCommands.registerCommand("RunConveyor", m_conveyorSubsystem.RunConveyorMM());
        NamedCommands.registerCommand("StopConveyor", m_conveyorSubsystem.StopConveyorMM());
        NamedCommands.registerCommand("RunKicker", m_kickerSubsystem.RunKickerMM());
        NamedCommands.registerCommand("StopKicker", m_kickerSubsystem.StopKickerMM());
        NamedCommands.registerCommand("ExtendClimber", m_climberSubsystem.ExtendClimberMM(2.3));
        NamedCommands.registerCommand("LowerClimber", m_climberSubsystem.LowerClimberMM(0.3));
        NamedCommands.registerCommand("ExtendHood", m_launcherSubsystem.ExtendHoodMM());
        NamedCommands.registerCommand("RetractHood", m_launcherSubsystem.RetractHoodMM());
        NamedCommands.registerCommand("AutoLaunchLookup", autoRunLaunchLookupWithTimeout());

        NamedCommands.registerCommand("Track", new TrackFieldPoseCommand(
                m_turretSubsystem,
                // Supplier<Pose2d>
                () -> drivetrain.getState().Pose,
                // Supplier<Translation2d> (FIELD-RELATIVE)
                (this::getFieldRelativeVelocity),
                TurretSubsystemConstants.ballSpeed));

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // Driver controls
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(
                                MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05) * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(
                                MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05) * MaxSpeed) // Drive left with
                                                                                                 // positive X (left)
                        .withRotationalRate(MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05) * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // X
                                                                                                                    // (left)
                ));

        // Finesse mode for precision control, activated by right bumper
        driverXbox.rightBumper().whileTrue(
                // Drivetrain will execute this command while right bumper is held
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(
                                MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05) * MaxSpeed * FinesseSpeedMult) // Drive
                                                                                                                    // forward
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // Y
                                                                                                                    // (forward)
                        .withVelocityY(
                                MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05) * MaxSpeed * FinesseSpeedMult) // Drive
                                                                                                                    // left
                                                                                                                    // with
                                                                                                                    // positive
                                                                                                                    // X
                                                                                                                    // (left)
                        .withRotationalRate(MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05) * MaxAngularRate
                                * FinesseAngularRateMult) // Drive counterclockwise with negative X (left)
                ));

        // Robot-centric control mode for precision maneuvering, activated by left
        // bumper
        driverXbox.leftBumper().whileTrue(
                // Drivetrain will execute this command while left bumper is held
                drivetrain.applyRequest(() -> forwardStraight
                        .withVelocityX(
                                MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05) * -MaxSpeed) // Drive forward with
                                                                                                  // negative Y
                                                                                                  // (forward)
                        .withVelocityY(
                                MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05) * -MaxSpeed) // Drive left with
                                                                                                  // negative X (left)
                        .withRotationalRate(
                                MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05) * -MaxAngularRate) // Drive
                                                                                                         // counterclockwise
                                                                                                         // with
                                                                                                         // negative X
                                                                                                         // (left)
                ));

        // D-pad for field-centric cardinal direction movement at half speed
        driverXbox.povUp()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight
                        .withVelocityX(0.5 * FinesseSpeedMult).withVelocityY(0)));
        driverXbox.povDown()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight
                        .withVelocityX(-0.5 * FinesseSpeedMult).withVelocityY(0)));
        driverXbox.povRight()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0)
                        .withVelocityY(-0.5 * FinesseSpeedMult)));
        driverXbox.povLeft()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0)
                        .withVelocityY(0.5 * FinesseSpeedMult)));

        driverXbox.start().onTrue((drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));
        driverXbox.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.leftTrigger().onTrue(m_intakeSubsystem.RunIntakeMM())
                .onFalse(m_intakeSubsystem.StopIntakeMM());
        driverXbox.y().onTrue(m_launcherSubsystem.IncrementLauncherSpeedUp()
                .andThen(m_launcherSubsystem.RunLauncherMM()));
        driverXbox.a().onTrue(m_launcherSubsystem.IncrementLauncherSpeedDown()
                .andThen(m_launcherSubsystem.RunLauncherMM()));
        driverXbox.x().onTrue(m_intakeSubsystem.StopIntakeMM());

        // COMPETITION AUX BINDINGS COMPETITION AUX BINDINGS COMPETITION AUX BINDINGS
        // COMPETITION AUX BINDINGS
        auxXbox.a().onTrue(new TrackFieldPoseCommand(
                m_turretSubsystem,
                // Supplier<Pose2d>
                () -> drivetrain.getState().Pose,
                // Supplier<Translation2d> (FIELD-RELATIVE)
                this::getFieldRelativeVelocity,
                TurretSubsystemConstants.ballSpeed));
        auxXbox.b().onTrue(m_turretSubsystem.StopTurret());

        auxXbox.y().onTrue(m_launcherSubsystem.RunLauncherMM()
                .alongWith(m_conveyorSubsystem.RunConveyorCommand())
                .alongWith(m_kickerSubsystem.RunKickerCommand())
                .alongWith(m_intakeSubsystem.RunIntakeMM()));

        auxXbox.x().onTrue(m_launcherSubsystem.StopLauncherMM()
                .alongWith(m_conveyorSubsystem.StopConveyorMM())
                .alongWith(m_kickerSubsystem.StopKickerMM())
                .alongWith(m_intakeSubsystem.StopIntakeMM()));

        auxXbox.povDown().onTrue(m_intakeSubsystem.DeployIntakeMM());
        auxXbox.povUp().onTrue(m_intakeSubsystem.RetractIntakeMM());

        auxXbox.rightTrigger().whileTrue(makeLaunchLookup());

        // auxXbox.x().whileTrue(makeLaunch());
        m_turretSubsystem.setDefaultCommand(
                m_turretSubsystem.run(() -> {
                    double joyX = -auxXbox.getRightX(); // left+
                    double joyY = -auxXbox.getRightY(); // forward+
                    double magnitude = Math.hypot(joyX, joyY);
                    if (magnitude > 0.5) {
                        // 0° = field forward, CCW+
                        double fieldAngleDeg = Math.toDegrees(Math.atan2(joyX, joyY));
                        // Convert field-relative angle to robot-relative turret angle
                        double robotHeading = drivetrain.getState().Pose.getRotation().getDegrees();
                        double turretAngle = fieldAngleDeg - robotHeading;
                        m_turretSubsystem.setTurretAngleDegrees(turretAngle);
                    }
                }));

        auxXbox.povRight().onTrue(m_launcherSubsystem.RetractHoodMM());

        auxXbox.povLeft().onTrue(m_turretSubsystem.ManualTurretPositionMM());
        auxXbox.leftBumper().onTrue(m_turretSubsystem.ManualTurretPositionLeftMM());
        auxXbox.rightBumper().onTrue(m_turretSubsystem.ManualTurretPositionRightMM());

        auxXbox.leftTrigger().onTrue(
                m_intakeSubsystem.reverseIntakeCommand()
        .alongWith(m_conveyorSubsystem.AgitateConveyorCommand())
        .alongWith(m_kickerSubsystem.AgitateKickerCommand())
        .alongWith(m_launcherSubsystem.agitateLauncherCommand())
        )
        .onFalse(
                Commands.runOnce(() -> m_conveyorSubsystem.stopConveyorMM(), m_conveyorSubsystem)
        .alongWith(Commands.runOnce(() -> m_kickerSubsystem.stopKickerMM(), m_kickerSubsystem))
        .alongWith(Commands.runOnce(() -> m_launcherSubsystem.stopLauncherMM(), m_launcherSubsystem))
        .alongWith(m_intakeSubsystem.StopIntakeCommand())
        );

        auxXbox.povLeft().onTrue(m_turretSubsystem.ManualTurretPositionMM());
        auxXbox.povLeft().onTrue(m_launcherSubsystem.MoveHoodUp().andThen(m_launcherSubsystem.ExtendHoodMM()));
        auxXbox.povRight().onTrue(m_launcherSubsystem.MoveHoodDown().andThen(m_launcherSubsystem.RetractHoodMM()));
        auxXbox.leftBumper().onTrue(m_turretSubsystem.ManualTurretPositionLeftMM());
        auxXbox.rightBumper().onTrue(m_turretSubsystem.ManualTurretPositionRightMM());


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

    private Translation2d getFieldRelativeVelocity() {
        ChassisSpeeds robotRelative = drivetrain.getState().Speeds;

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelative,
                drivetrain.getState().Pose.getRotation());

        return new Translation2d(
                fieldRelative.vxMetersPerSecond,
                fieldRelative.vyMetersPerSecond);
    }
}
