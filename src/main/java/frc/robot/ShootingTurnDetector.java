package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Detects whether it is our alliance's shooting turn during the 2026 REBUILT
 * game's shift-based teleop structure.
 *
 * <h3>2026 REBUILT Shift Timing (teleop match-time countdown)</h3>
 * <pre>
 *   > 130 s  →  Transition Shift  – both hubs active  →  always our turn
 *   105–130s  →  Shift 1           – auto loser's hub active
 *    80–105s  →  Shift 2           – auto winner's hub active
 *    55– 80s  →  Shift 3           – auto loser's hub active
 *    30– 55s  →  Shift 4           – auto winner's hub active
 *      ≤ 30s  →  End Game          – both hubs active  →  always our turn
 * </pre>
 *
 * <h3>Game Specific Message (GSM)</h3>
 * FMS sends a single character ({@code 'R'} or {@code 'B'}) ~3 s after auto ends.
 * It identifies the alliance whose hub goes <em>inactive first</em>: that
 * alliance is active in Shifts 2 &amp; 4; the other alliance is active in
 * Shifts 1 &amp; 3.
 *
 * <h3>Human-player fallback</h3>
 * If the GSM is not yet available (common during the Transition Shift),
 * the human player can set {@code ShootingTurn/HumanPlayerOverride} to
 * {@code true} on the SmartDashboard / driver-station dashboard to manually
 * declare that it is our turn.
 *
 * <h3>Usage</h3>
 * <pre>
 *   // In RobotContainer constructor:
 *   ShootingTurnDetector detector = new ShootingTurnDetector();
 *   detector.shootingTurn().whileTrue(makeLaunchLookup());
 *
 *   // In Robot.robotPeriodic():
 *   m_robotContainer.m_shootingTurnDetector.update();
 * </pre>
 */
public class ShootingTurnDetector {

    /** SmartDashboard key the human player can set to {@code true} when the
     *  Game Specific Message has not arrived yet. */
    public static final String kHumanOverrideKey = "ShootingTurn/HumanPlayerOverride";

    private boolean m_isOurTurn   = false;
    private boolean m_gsmAvailable = false;
    private String  m_shiftName   = "—";

    public ShootingTurnDetector() {
        // Put the override key on SmartDashboard so it appears in the dashboard widget list.
        SmartDashboard.putBoolean(kHumanOverrideKey, false);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Returns a {@link Trigger} that is {@code true} whenever it is our
     * alliance's shooting turn.  Bind launch commands to this in
     * {@code RobotContainer.configureBindings()}.
     */
    public Trigger shootingTurn() {
        return new Trigger(this::isOurTurn);
    }

    /** @return {@code true} when it is currently our alliance's shooting turn. */
    public boolean isOurTurn() {
        return m_isOurTurn;
    }

    /**
     * Update internal state.  Must be called every robot loop
     * (from {@code Robot.robotPeriodic()}).
     */
    public void update() {
        if (!DriverStation.isEnabled() || !DriverStation.isTeleop()) {
            m_isOurTurn = false;
            m_shiftName = "NotTeleop";
            publish();
            return;
        }

        double matchTime = DriverStation.getMatchTime();

        // Transition Shift OR FMS not reporting time → both hubs active
        if (matchTime < 0 || matchTime > 130) {
            m_isOurTurn = true;
            m_shiftName = "Transition";
            publish();
            return;
        }

        // End Game → both hubs active
        if (matchTime <= 30) {
            m_isOurTurn = true;
            m_shiftName = "EndGame";
            publish();
            return;
        }

        // ── Shifts 1–4: use GSM or human-player fallback ─────────────────────
        String gsm = DriverStation.getGameSpecificMessage();
        m_gsmAvailable = (gsm != null && !gsm.isEmpty());

        if (!m_gsmAvailable) {
            // GSM not yet delivered — ask the human player
            m_isOurTurn = SmartDashboard.getBoolean(kHumanOverrideKey, false);
            m_shiftName = "GSM_Pending_HumanOverride";
            publish();
            return;
        }

        // Determine our alliance character
        boolean isBlue = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        char allianceChar = isBlue ? 'B' : 'R';

        // GSM character = alliance whose hub goes inactive *first*
        // → that alliance's hub is active in Shifts 2 & 4 (even shifts)
        // → the OTHER alliance's hub is active in Shifts 1 & 3 (odd shifts)
        boolean weAreEvenShiftAlliance = (gsm.charAt(0) == allianceChar);

        int shiftNumber;
        if      (matchTime > 105) { shiftNumber = 1; }
        else if (matchTime >  80) { shiftNumber = 2; }
        else if (matchTime >  55) { shiftNumber = 3; }
        else                      { shiftNumber = 4; }

        m_shiftName = "Shift" + shiftNumber;

        boolean isEvenShift = (shiftNumber % 2 == 0);
        m_isOurTurn = (isEvenShift == weAreEvenShiftAlliance);

        publish();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private void publish() {
        SmartDashboard.putBoolean("ShootingTurn/IsOurTurn",    m_isOurTurn);
        SmartDashboard.putBoolean("ShootingTurn/GSMAvailable", m_gsmAvailable);
        SmartDashboard.putString ("ShootingTurn/CurrentShift", m_shiftName);
        SmartDashboard.putNumber ("ShootingTurn/MatchTimeSec", DriverStation.getMatchTime());
    }
}
