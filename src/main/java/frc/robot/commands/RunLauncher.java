package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** Command to run two independent Kraken motors using Motion Magic velocity. */
public class RunLauncher extends Command {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LauncherSubsystem m_subsystem;

    // Suppliers for each motor's target velocity (RPS)
    private final DoubleSupplier m_launcherSupplier;
    private final DoubleSupplier m_angleSupplier;

    /**
     * Creates a new RunDualKrakenMotors command.
     *
     * @param subsystem The dual Kraken motion magic subsystem
     * @param launcherSupplier Supplier for Motor A velocity (RPS)
     * @param angleSupplier Supplier for Motor B velocity (RPS)
     */
    public RunLauncher(
            LauncherSubsystem subsystem,
            DoubleSupplier launcherSupplier,
            DoubleSupplier angleSupplier) {

        m_subsystem = subsystem;
        m_launcherSupplier = launcherSupplier;
        m_angleSupplier = angleSupplier;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        updateMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updateMotors();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (m_launcherSupplier != null) {
            m_subsystem.setLauncherVelocity(0.0);
        }
        if (m_angleSupplier != null) {
            // Optionally leave at last position or stop
        }
      }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

private void updateMotors() {
        if (m_launcherSupplier != null) {
            m_subsystem.setLauncherVelocity(m_launcherSupplier.getAsDouble());
        }
        if (m_angleSupplier != null) {
            m_subsystem.setAnglePosition(m_angleSupplier.getAsDouble());
        }
    }

}
