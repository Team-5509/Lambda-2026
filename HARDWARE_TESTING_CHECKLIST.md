# Robot Hardware Testing Checklist

Follow this order when testing with the physical robot. Check off each item as it passes.

---

## 1. Deploy Intake

**Hardware:** TalonFX (CAN 21), CANcoder (CAN 22)

- [ ] CAN 21 (deploy motor) appears on CAN bus with no faults
- [ ] CAN 22 (deploy encoder) appears on CAN bus and reads absolute position
- [ ] `DeployIntake` command moves intake to deployed position smoothly
- [ ] `RetractIntake` command moves intake to retracted position smoothly
- [ ] Soft limit MIN (−0.5 rot) stops motion before mechanical hard stop
- [ ] Soft limit MAX (+0.5 rot) stops motion before mechanical hard stop
- [ ] Motion Magic profile runs without oscillation (cruise 2.0 rot/s, accel 6.0 rot/s²)
- [ ] Position holds when motor is at rest

---

## 2. Intake

**Hardware:** TalonFX (CAN 14)

- [ ] CAN 14 (intake motor) appears on CAN bus with no faults
- [ ] `RunIntake` command spins intake at default speed (100.0)
- [ ] `StopIntake` command stops intake motor cleanly
- [ ] Increment speed up/down buttons change speed by 10.0 per press
- [ ] Intake rollers spin in the correct direction to collect game pieces
- [ ] Game piece is successfully collected and passed toward the conveyor

---

## 3. Conveyor

**Hardware:** TalonFX (CAN 15)

- [ ] CAN 15 (conveyor motor) appears on CAN bus with no faults
- [ ] `RunConveyor` command spins conveyor at default speed (100.0)
- [ ] `StopConveyor` command stops conveyor motor cleanly
- [ ] Increment speed up/down buttons change speed by 10.0 per press
- [ ] Conveyor belt/roller spins in the correct direction to transport game pieces
- [ ] Game piece travels from intake through conveyor to kicker zone

---

## 4. Kicker

**Hardware:** TalonFX (CAN 16)

- [ ] CAN 16 (kicker motor) appears on CAN bus with no faults
- [ ] `RunKicker` command spins kicker at default speed (100.0)
- [ ] `StopKicker` command stops kicker motor cleanly
- [ ] Increment speed up/down buttons change speed by 10.0 per press
- [ ] Tuned gains (kP=0.36, kV=0.145, kA=0.15) give consistent velocity
- [ ] Kicker ejects game piece into launcher path without jamming
- [ ] Full game piece path works: intake → conveyor → kicker → launcher

---

## 5. Turret

**Hardware:** TalonFX (CAN 17), Limit Switch Negative (DIO 0), Limit Switch Positive (DIO 1)

- [ ] CAN 17 (turret motor) appears on CAN bus with no faults
- [ ] DIO 0 (negative limit) reads correctly in SmartDashboard
- [ ] DIO 1 (positive limit) reads correctly in SmartDashboard
- [ ] `HomeTurret` command rotates turret toward negative limit and zeros position on contact
- [ ] DIO 0 triggers and **stops** motion at the negative end of travel
- [ ] DIO 1 triggers and **stops** motion at the positive end of travel
- [ ] Soft limit MIN (−0.75 rot) triggers before mechanical hard stop
- [ ] Soft limit MAX (+0.25 rot) triggers before mechanical hard stop
- [ ] SmartDashboard `TurretSubsystem/TurretPosition` tracks real position accurately
- [ ] **TODO — Measure:** actual min rotation angle → update `MIN_TURRET_ROT` in Constants.java (line 72)
- [ ] **TODO — Measure:** actual max rotation angle → update `MAX_TURRET_ROT` in Constants.java (line 73)
- [ ] **TODO — Confirm:** gear ratio 5000/120 ≈ 41.67:1 matches physical gearbox
- [ ] Motion Magic profile runs without oscillation (kP=60.0, kD=5.0)

---

## 6. Launcher

**Hardware:** TalonFX (CAN 18)

- [ ] CAN 18 (launcher flywheel) appears on CAN bus with no faults
- [ ] `RunLauncher` command spins flywheel up to commanded RPS
- [ ] `StopLauncher` command coasts/stops flywheel cleanly
- [ ] kV=0.125 feed-forward gives accurate no-load speed
- [ ] SmartDashboard `LauncherSubsystem/LauncherRPS` matches setpoint within tolerance
- [ ] Spin-up delay of 0.2 s is sufficient before firing
- [ ] Increment speed up/down buttons change speed by 10.0 RPS per press
- [ ] **TODO — Measure:** actual flywheel diameter (currently 4 in / 0.1016 m) → update `kLauncherWheelDiameterM` in Constants.java (line 100)

---

## 7. Hood

**Hardware:** TalonFX (CAN 19), CANcoder (CAN 23), Limit Switch Low (DIO 4)

- [ ] CAN 19 (hood motor) appears on CAN bus with no faults
- [ ] CAN 23 (hood encoder) appears on CAN bus and reads absolute position
- [ ] DIO 4 (low limit switch) reads correctly in SmartDashboard
- [ ] Hood moves up (`MoveHoodUp`) and down (`MoveHoodDown`) without binding
- [ ] `RetractHoodMM` moves hood to minimum position and triggers DIO 4
- [ ] DIO 4 contact resets hood position to 0.05 rot (home zeroing works)
- [ ] `ExtendHoodMM` moves hood to maximum position cleanly
- [ ] Soft limit MIN (0.05 rot) stops motion before mechanical hard stop
- [ ] Soft limit MAX (1.70 rot) stops motion before mechanical hard stop
- [ ] Gravity compensation (Arm_Cosine, kG=−0.04) holds hood angle without drift
- [ ] CANcoder offset −0.186523 is correct — hood reads 0.05 rot at physical home
- [ ] SmartDashboard `LauncherSubsystem/HoodPosition` tracks real position accurately
- [ ] **TODO — Measure:** actual minimum hood angle (est. 20°) → update `kHoodMinAngleDeg` in Constants.java (line 95)
- [ ] **TODO — Measure:** actual maximum hood angle (est. 70°) → update `kHoodMaxAngleDeg` in Constants.java (line 96)

---

## 8. Shooting (Turret + Launcher + Hood combined)

**Depends on:** steps 5, 6, and 7 passing first

- [ ] `TrackFieldPoseCommand` aims turret at hub when on own side of field
- [ ] `TrackFieldPoseCommand` aims turret at nearest home goal when on opponent side
- [ ] SmartDashboard `TrackFieldPoseCommand/Target` shows correct target coordinates
- [ ] **TODO — Measure & update:** shooter height from ground (est. 21.5 in) in Constants.java (line 55)
- [ ] **TODO — Measure & update:** home goal height (est. 24.0 in) in Constants.java (line 60)
- [ ] **TODO — Measure & update:** shooter exit offset from robot center (est. X=0.32 m, Y=0.18 m) in Constants.java (line 63–64)
- [ ] `Launch` command (physics-based): calculated hood angle is within 20°–70° range
- [ ] `Launch` command: SmartDashboard `Launch/CalculatedHoodAngleDeg` and `Launch/CalculatedLauncherRPS` are reasonable values
- [ ] **TODO — Calibrate:** `kHoodMinAngleDeg` and `kHoodMaxAngleDeg` in Launch.java (line 187) after hood angles are measured
- [ ] `LaunchLookup` command: hub lookup table returns plausible hood/speed values at test distances (1.5 m–6.5 m)
- [ ] `LaunchLookup` command: home goal lookup table returns plausible values
- [ ] **TODO — Fill tables:** replace placeholder values in LaunchLookup.java (lines 53–84) with measured shot data at each distance
- [ ] Full shot cycle: spin up → aim turret → set hood → fire → game piece reaches target
- [ ] Repeat shots at multiple distances for consistency

---

## 9. Climb

**Hardware:** TalonFX (CAN 20)

- [ ] CAN 20 (climber motor) appears on CAN bus with no faults
- [ ] POV Up on aux Xbox controller triggers `ExtendClimberMM` and extends climber
- [ ] POV Down on aux Xbox controller triggers `LowerClimberMM` and retracts climber
- [ ] Brake mode holds climber position when controller is released (does not back-drive)
- [ ] Motion Magic profile runs smoothly (cruise 2.0 rot/s, accel 6.0 rot/s²)
- [ ] Full extension reaches expected endpoint without hitting hard stop
- [ ] Full retraction reaches expected endpoint without hitting hard stop
- [ ] Robot can support weight on extended climber (static load test)

---

## 10. Auton

**Depends on:** all subsystems above passing, drivetrain and vision passing (see Miscellaneous)

- [ ] AutoBuilder initializes without errors; "Auto Mode" chooser appears on SmartDashboard
- [ ] Default auto "PlsDontExplode" is selectable and runs without crashing
- [ ] Named commands execute in correct sequence within a path (RunIntake, DeployIntake, Track, Launch, etc.)
- [ ] `AutoCloseHang` — full sequence: shoot middle → intake → shoot close wall → climb
- [ ] `AutoCloseNoHang` — full sequence: shoot middle → intake → shoot close wall (no climb)
- [ ] `LeftTrenchNoHang` — full sequence: trench intake path completes
- [ ] `LeftTrenchHang` — full sequence: trench intake + climb completes
- [ ] `RightTrenchNoHang` — full sequence completes
- [ ] `RightTrenchHang` — full sequence + climb completes
- [ ] Odometry remains accurate through multi-segment paths (robot ends near expected pose)
- [ ] Vision pose corrections apply correctly during auto

---

## Miscellaneous

Items from the codebase not covered by the 10 sections above.

### Drivetrain (Swerve Drive)

**Hardware:** 4× Drive TalonFX (CAN 1,3,5,7), 4× Steer TalonFX (CAN 2,4,6,8), 4× CANcoder (CAN 9,10,11,12), Pigeon 2 IMU (CAN 13)

- [ ] CAN 13 (Pigeon 2 IMU) reads heading; field-relative reset works
- [ ] All 8 drive/steer TalonFX devices appear on CAN bus with no faults
- [ ] All 4 CANcoders appear on CAN bus and read absolute steer position
- [ ] Verify swerve encoder offsets match physical wheel-straight positions:
  - [ ] Front Left (CAN 9): 0.300537 rot
  - [ ] Front Right (CAN 10): −0.039063 rot
  - [ ] Back Left (CAN 11): −0.244873 rot
  - [ ] Back Right (CAN 12): 0.350098 rot
- [ ] Each module steers to 0° correctly when zeroed
- [ ] Full swerve drive: forward, strafe, and rotation without wheel scrub
- [ ] Max speed at 12 V ≈ 5.12 m/s achieved in a straight-line test
- [ ] Odometry tracks position accurately across the field

### Vision (PhotonVision — AprilTag)

**Hardware:** 4 cameras ("Apple", "Basil", "Dragonfruit", "Banana")

- [ ] "Apple" (front-left, CAN +45° pan) — connects to NetworkTables, detects tags
- [ ] "Basil" (front-right, −45° pan) — connects to NetworkTables, detects tags
- [ ] "Dragonfruit" (rear-left, +135° pan) — connects to NetworkTables, detects tags
- [ ] "Banana" (rear-right, −135° pan) — connects to NetworkTables, detects tags
- [ ] Multi-tag PNP pose estimates are within ~10 cm of known robot position
- [ ] Vision measurements fuse into odometry without large jumps
- [ ] **TODO — Verify:** all 4 camera mount positions/rotations match constants in CameraManager.java
- [ ] Camera resolution 960×720 @ 15 FPS stable under field lighting

### LEDs (CANdle)

**Hardware:** CANdle (CAN 1), 392-LED external strip (GRB)

- [ ] CAN 1 (CANdle) appears on CAN bus with no faults
- [ ] Onboard LEDs 0–7 light the correct color on command
- [ ] External strip LEDs 8–399 (392 LEDs, GRB) all illuminate — no dead sections
- [ ] Brightness scalar 50% applied correctly (no full-brightness flicker)
- [ ] LED state changes match robot state (e.g., color changes on intake, shooting, etc.)
