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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.Constants.TurretSubsystemConstants;
import frc.robot.subsystems.ConveyorSubsystem;

public class RobotContainer {
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
    private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    //private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
 // CCW+, field-relative

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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

    // This command will track the robot's pose on the field using vision
    // measurements and drive the turret to point at the hub
    Command trackHub = new TrackFieldPoseCommand(
            m_turretSubsystem,
            // Supplier<Pose2d>
            () -> drivetrain.getState().Pose,
            // Supplier<Translation2d> (FIELD-RELATIVE)
            this::getFieldRelativeVelocity,

            TurretSubsystemConstants.hubPose,

            TurretSubsystemConstants.ballSpeed);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("PlsDontExplode");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // Driver controls
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Set the default command for the drivetrain. The drivetrain will automatically
        // run the default command whenever no other command is using the drivetrain
        // subsystem. This is where we set up the default behavior for the drivetrain,
        // which is to drive with the left and right sticks of the Xbox controller, and
        // to track the hub with vision. The applyRequest method is used to create a
        // command that applies the given SwerveRequest to the drivetrain. The
        // withVelocityX, withVelocityY, and withRotationalRate methods are used to
        // specify the desired velocities for the drivetrain based on the Xbox
        // controller inputs. The MathUtil.applyDeadband method is used to apply a
        // deadband to the controller inputs, which helps to prevent small joystick
        // movements from causing the robot to move. The MaxSpeed and MaxAngularRate
        // variables are used to scale the joystick inputs to the desired maximum speeds
        // for the drivetrain. The andThen(trackHub) part means that after applying the
        // drive request, the drivetrain will also execute the trackHub command, which
        // will use vision to track the hub and adjust the turret accordingly.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive
                                .withVelocityX(
                                        -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.05)
                                                * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.05)
                                                * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(
                                        -MathUtil.applyDeadband(driverXbox.getRightX(), 0.05)
                                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ).andThen(trackHub));

        driverXbox.y().onTrue((drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));
        driverXbox.x().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.b().onTrue(drivetrain.runOnce(() -> drivetrain.addFakeVisionReading()));

        driverXbox.povUp()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driverXbox.povDown()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        driverXbox.povRight()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));
        driverXbox.povLeft()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));

        // Aux driver controls

        //Conveyor
        auxXbox.b().onTrue(m_conveyorSubsystem.RunConveyorMM());
        auxXbox.x().onTrue(m_conveyorSubsystem.StopConveyorMM());
        auxXbox.povRight().onTrue(m_conveyorSubsystem.IncrementConveyorSpeedUp().andThen(m_conveyorSubsystem.RunConveyorMM()));
        auxXbox.povLeft().onTrue(m_conveyorSubsystem.IncrementConveyorSpeedDown().andThen(m_conveyorSubsystem.RunConveyorMM()));

        //Kicker
        auxXbox.a().onTrue(m_kickerSubsystem.RunKickerMM());
        auxXbox.y().onTrue(m_kickerSubsystem.StopKickerMM());
        auxXbox.povUp().onTrue(m_kickerSubsystem.IncrementKickerSpeedUp().andThen(m_kickerSubsystem.RunKickerMM()));
        auxXbox.povDown().onTrue(m_kickerSubsystem.IncrementKickerSpeedDown().andThen(m_kickerSubsystem.RunKickerMM()));

        //Intake
        auxXbox.rightBumper().onTrue(m_intakeSubsystem.RunIntakeMM());
        auxXbox.leftBumper().onTrue(m_intakeSubsystem.StopIntakeMM());
        auxXbox.povUp().onTrue(m_intakeSubsystem.IncrementIntakeSpeedUp().andThen(m_intakeSubsystem.RunIntakeMM()));
        auxXbox.povDown().onTrue(m_intakeSubsystem.IncrementIntakeSpeedDown().andThen(m_intakeSubsystem.RunIntakeMM()));
        auxXbox.a().onTrue(m_intakeSubsystem.DeployIntakeMM());
        auxXbox.y().onTrue(m_intakeSubsystem.RetractIntakeMM());
        auxXbox.povUp().onTrue(m_intakeSubsystem.IncrementDeployPositionUp().andThen(m_intakeSubsystem.DeployIntakeMM()));
        auxXbox.povDown().onTrue(m_intakeSubsystem.IncrementDeployPositionDown().andThen(m_intakeSubsystem.DeployIntakeMM()));

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
