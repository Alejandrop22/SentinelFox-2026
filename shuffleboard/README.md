# Shuffleboard (Match Dashboard)

This folder is for sharing a **single-screen Match dashboard** layout with the whole team.

## What everyone needs
- No extra downloads.
- Just WPILib Driver Station + Shuffleboard (already included in the FRC tools).

## How to export the layout (one person sets it up)
1. Open **Shuffleboard**.
2. Arrange widgets on one tab (recommended tab name: `Match`).
3. In Shuffleboard: **File → Save** (or **Save As**) and save the file into this folder.

Recommended filename:
- `shuffleboard/match-layout.json`

Commit/push that file so everyone else can pull it.

## How to import the layout (everyone else)
1. Pull the repo.
2. Open **Shuffleboard**.
3. **File → Open** and select `shuffleboard/match-layout.json`.

## Suggested widgets (1 screen)
Use these NetworkTables keys (already published by the robot code):

### Camera / Vision
- `Vision/HasTargets` (boolean)
- `Vision/HasTag1` (boolean)
- `Vision/Tag1YawDeg` (number)
- `Vision/DistanceM` (number)

### AutoAim
- `AutoAim/Enabled` (boolean)
- `AutoAim/TrackingTag1` (boolean)
- `AutoAim/YawErrorDeg` (number)
- `AutoAim/RotCmd` (number)

### Shooter
- `Shooter/AssistedEnabled` (boolean)
- `Shooter/EmergencyEnabled` (boolean)
- `Shooter/VelocityRPM` (number)
- `Shooter/AppliedPercent` (number)

### Assisted Shooter (debug)
- `AssistShooter/DistanceM` (number)
- `AssistShooter/Percent` (number)

### Drive
- `Drive/HeadingDeg` (number)
- `Drive/TurnRateDps` (number)
- `Drive/PoseX` (number)
- `Drive/PoseY` (number)
- `Drive/PoseDeg` (number)

## Notes
- If a widget shows `?`, it usually means the robot code isn’t connected/publishing yet.
- Keep key names stable so the saved layout keeps working for everyone.
