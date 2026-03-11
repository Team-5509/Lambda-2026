package frc.robot.commands;

import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Slowly rotates the turret clockwise until a limit switch is reached.
 * Motor is configured CounterClockwise_Positive, so clockwise = negative speed.
 * Clockwise rotation heads toward the negative limit switch at -270°.
 */
public class SlowClockwiseToSwitch extends Command {
    private static final double SLOW_CW_SPEED = -0.1;

    private final TurretSubsystem m_turret;

    public SlowClockwiseToSwitch(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_turret.setSpeed(SLOW_CW_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }

    @Override
    public boolean isFinished() {
        return m_turret.isAtNegativeLimit() || m_turret.isAtPositiveLimit();
    }
}
