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
 * <h3>Shift offsets</h3>
 * The second constructor accepts a {@code frontOffset} and {@code backOffset}
 * (in seconds) to shrink the effective shooting window within each shift:
 * <ul>
 *   <li><b>frontOffset</b> – ignore the first N seconds of a shift (robot
 *       may still be repositioning after the hub activates).</li>
 *   <li><b>backOffset</b> – ignore the last N seconds of a shift (don't
 *       start a shot that may not finish before the hub deactivates).</li>
 * </ul>
 * Offsets are <em>not</em> applied during the always-active Transition Shift
 * or End Game periods.
 *
 * <h3>Game Specific Message (GSM)</h3>
 * FMS sends a single character ({@code 'R'} or {@code 'B'}) ~3 s after auto
 * ends.  It identifies the alliance whose hub goes <em>inactive first</em>:
 * that alliance is active in Shifts 2 &amp; 4; the other alliance is active
 * in Shifts 1 &amp; 3.
 *
 * <h3>Human-player fallback</h3>
 * If the GSM is not yet available the human player can set
 * {@code ShootingTurn/HumanPlayerOverride} to {@code true} on the
 * SmartDashboard to manually declare that it is our turn.
 *
 * <h3>Usage</h3>
 * <pre>
 *   // No offsets:
 *   ShootingTurnDetector detector = new ShootingTurnDetector();
 *
 *   // With 1 s front buffer and 2 s back buffer:
 *   ShootingTurnDetector detector = new ShootingTurnDetector(1.0, 2.0);
 *
 *   // Bind to commands in RobotContainer.configureBindings():
 *   detector.shootingTurn().whileTrue(makeLaunchLookup());
 *   detector.shootingTurn(1.5).whileTrue(makeLaunchLookup()); // needs 1.5 s to shoot
 *
 *   // Must be called every loop in Robot.robotPeriodic():
 *   m_robotContainer.m_shootingTurnDetector.update();
 * </pre>
 */
public class ShootingTurnDetector {

    /** SmartDashboard key the human player can set to {@code true} when the
     *  Game Specific Message has not yet arrived. */
    public static final String kHumanOverrideKey = "ShootingTurn/HumanPlayerOverride";

    private final double m_frontOffset; // seconds to wait into a shift before shooting
    private final double m_backOffset;  // seconds before shift end to stop shooting

    // Updated every loop by update()
    private boolean m_rawOurShift   = false; // true when our hub is the active one (no offsets)
    private boolean m_alwaysActive  = false; // true during Transition / End Game (both hubs on)
    private double  m_shiftStart    = 0;     // matchTime value where current shift began (higher)
    private double  m_shiftEnd      = 0;     // matchTime value where current shift ends (lower)
    private boolean m_gsmAvailable  = false;
    private String  m_shiftName     = "—";

    // ── Constructors ──────────────────────────────────────────────────────────

    /** No shift-boundary offsets. */
    public ShootingTurnDetector() {
        this(0.0, 0.0);
    }

    /**
     * @param frontOffset seconds to wait into each shift before considering it
     *                    our turn (robot repositioning buffer at shift start).
     * @param backOffset  seconds before each shift ends to stop considering it
     *                    our turn (prevents starting a shot that won't finish).
     */
    public ShootingTurnDetector(double frontOffset, double backOffset) {
        m_frontOffset = frontOffset;
        m_backOffset  = backOffset;
        SmartDashboard.putBoolean(kHumanOverrideKey, false);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Returns a {@link Trigger} that is {@code true} whenever it is our
     * alliance's shooting turn (respecting front/back offsets).
     */
    public Trigger shootingTurn() {
        return new Trigger(this::isOurTurn);
    }

    /**
     * Returns a {@link Trigger} that is {@code true} when it is our turn AND
     * there is at least {@code shootTime} seconds remaining in the current
     * shift (after the back-offset buffer).  Use this to avoid starting a
     * shot that won't finish before the hub deactivates.
     *
     * @param shootTime estimated seconds needed to complete one shot cycle
     */
    public Trigger shootingTurn(double shootTime) {
        return new Trigger(() -> isOurTurn(shootTime));
    }

    /**
     * @return {@code true} when it is currently our alliance's shooting turn,
     *         accounting for {@code frontOffset} and {@code backOffset}.
     */
    public boolean isOurTurn() {
        if (!m_rawOurShift) return false;
        if (m_alwaysActive) return true; // Transition / End Game: no offset applied

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) return true; // no FMS time — assume OK

        double elapsed   = m_shiftStart - matchTime; // seconds since shift started
        double remaining = matchTime - m_shiftEnd;   // seconds until shift ends
        return elapsed >= m_frontOffset && remaining >= m_backOffset;
    }

    /**
     * @param shootTime estimated seconds needed to complete one shot cycle
     * @return {@code true} when it is our turn AND a shot started now would
     *         finish before the shift ends (with the back-offset buffer).
     */
    public boolean isOurTurn(double shootTime) {
        if (!isOurTurn()) return false;
        if (m_alwaysActive) {
            // During always-active periods just check overall match time
            double matchTime = DriverStation.getMatchTime();
            return matchTime < 0 || matchTime >= shootTime;
        }
        // Time available = remaining in shift minus the back-offset buffer
        double matchTime   = DriverStation.getMatchTime();
        if (matchTime < 0) return true;
        double available   = (matchTime - m_shiftEnd) - m_backOffset;
        return available >= shootTime;
    }

    /**
     * Update internal state.  Must be called every robot loop
     * (from {@code Robot.robotPeriodic()}).
     */
    public void update() {
        if (!DriverStation.isEnabled() || !DriverStation.isTeleop()) {
            m_rawOurShift  = false;
            m_alwaysActive = false;
            m_shiftName    = "NotTeleop";
            publish();
            return;
        }

        double matchTime = DriverStation.getMatchTime();

        // Transition Shift OR FMS not reporting time → both hubs active
        if (matchTime < 0 || matchTime > 130) {
            m_rawOurShift  = true;
            m_alwaysActive = true;
            m_shiftStart   = 140;
            m_shiftEnd     = 130;
            m_shiftName    = "Transition";
            publish();
            return;
        }

        // End Game → both hubs active
        if (matchTime <= 30) {
            m_rawOurShift  = true;
            m_alwaysActive = true;
            m_shiftStart   = 30;
            m_shiftEnd     = 0;
            m_shiftName    = "EndGame";
            publish();
            return;
        }

        m_alwaysActive = false;

        // ── Shifts 1–4: use GSM or human-player fallback ─────────────────────
        String gsm = DriverStation.getGameSpecificMessage();
        m_gsmAvailable = (gsm != null && !gsm.isEmpty());

        if (!m_gsmAvailable) {
            m_rawOurShift = SmartDashboard.getBoolean(kHumanOverrideKey, false);
            m_shiftName   = "GSM_Pending_HumanOverride";
            // Keep previous shift boundary estimates so shootTime checks still work
            resolveShiftBounds(matchTime);
            publish();
            return;
        }

        boolean isBlue = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        char allianceChar = isBlue ? 'B' : 'R';

        // GSM char = alliance whose hub goes inactive first → active in even shifts (2 & 4)
        boolean weAreEvenShiftAlliance = (gsm.charAt(0) == allianceChar);

        resolveShiftBounds(matchTime);

        int shiftNumber = shiftNumberFor(matchTime);
        m_shiftName = "Shift" + shiftNumber;

        boolean isEvenShift = (shiftNumber % 2 == 0);
        m_rawOurShift = (isEvenShift == weAreEvenShiftAlliance);

        publish();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private static int shiftNumberFor(double matchTime) {
        if      (matchTime > 105) return 1;
        else if (matchTime >  80) return 2;
        else if (matchTime >  55) return 3;
        else                      return 4;
    }

    /** Sets {@code m_shiftStart} / {@code m_shiftEnd} for the current shift. */
    private void resolveShiftBounds(double matchTime) {
        if      (matchTime > 105) { m_shiftStart = 130; m_shiftEnd = 105; }
        else if (matchTime >  80) { m_shiftStart = 105; m_shiftEnd =  80; }
        else if (matchTime >  55) { m_shiftStart =  80; m_shiftEnd =  55; }
        else                      { m_shiftStart =  55; m_shiftEnd =  30; }
    }

    private void publish() {
        SmartDashboard.putBoolean("ShootingTurn/IsOurTurn",    isOurTurn());
        SmartDashboard.putBoolean("ShootingTurn/GSMAvailable", m_gsmAvailable);
        SmartDashboard.putString ("ShootingTurn/CurrentShift", m_shiftName);
        SmartDashboard.putNumber ("ShootingTurn/MatchTimeSec", DriverStation.getMatchTime());
        SmartDashboard.putNumber ("ShootingTurn/ShiftRemainingSec",
                DriverStation.getMatchTime() >= 0 ? DriverStation.getMatchTime() - m_shiftEnd : -1);
    }
}
