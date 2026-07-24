# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

The FTC (FIRST Tech Challenge) SDK for the DECODE (2025–2026) season — an Android app (`com.qualcomm.ftcrobotcontroller`) that runs on a Control Hub or Android phone and drives a competition robot. The SDK itself is consumed as Maven artifacts (`org.firstinspires.ftc:*:11.1.0`); this repo is the *team-modifiable shell* around it.

Requires Android Studio Ladybug (2024.2) or later. JDK 21 (bundled with Ladybug) is used, but `sourceCompatibility`/`targetCompatibility` are pinned to Java 1.8 so code stays runnable in the on-robot OnBotJava environment. Don't raise these.

## Build / install / run

There is no test suite — verification is "does it build" and "does it run on the robot." Use the Gradle wrapper:

```bash
./gradlew assembleDebug              # build debug APK for the robot controller
./gradlew :TeamCode:assembleDebug    # build only the team module (faster iteration)
./gradlew installDebug               # build + install on attached Android device via adb
./gradlew clean
```

Typical inner loop is to use Android Studio's Run button against an attached Control Hub / phone over USB or ADB-over-WiFi. The deployable artifact is the `FtcRobotController` app; `TeamCode` is an Android library that gets linked into it.

## Module layout — what goes where

Two-module Gradle build (see `settings.gradle`):

- **`FtcRobotController/`** — the Android *application* module. **Do not modify team code here.** Contains:
  - `internal/` — the actual app entry points (`FtcRobotControllerActivity`, `FtcOpModeRegister`, `PermissionValidatorWrapper`). Modify only if you really know what you're doing.
  - `external/samples/` — read-only reference OpModes (63 files). **Never edit these in place.** The workflow is: copy a sample into `TeamCode/.../teamcode/` (Android Studio's copy-paste auto-renames the class), then modify the copy. See `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/readme.md` for the sample-naming convention (`Basic*`, `Sensor*`, `Robot*`, `Concept*`).
- **`TeamCode/`** — Android *library* module, package `org.firstinspires.ftc.teamcode`. **This is where all team code goes.** Has `OpModeAnnotationProcessor.jar` in `TeamCode/lib/` that picks up `@TeleOp` / `@Autonomous` annotations at compile time and registers OpModes for the Driver Station menu.

`build.common.gradle` is shared build config and reads `versionCode` / `versionName` from `FtcRobotController/src/main/AndroidManifest.xml` — that manifest is the single source of truth for app version. `build.dependencies.gradle` pins the FTC SDK artifact versions (currently `11.1.0`).

## OpMode model (the runtime abstraction)

OpModes are the unit of robot behavior. An OpMode is a Java class annotated with `@TeleOp(name=..., group=...)` or `@Autonomous(...)` that extends `LinearOpMode` (linear/imperative style with `waitForStart()` + `while (opModeIsActive())`) or `OpMode` (iterative `init()`/`loop()` style). The annotation processor auto-discovers them — no manual registration needed. Adding `@Disabled` hides an OpMode from the Driver Station menu.

Hardware is accessed via `hardwareMap.get(DcMotor.class, "name")` where `"name"` must match a port name configured in the FTC Robot Controller app's robot configuration on the device — these strings are a contract with on-device configuration, not the codebase, so renaming them in code without updating the robot's config will break runtime hardware lookup.

## ChargedCreeper robot — hardware config and conventions

> **OpMode walkthroughs:** [`OLD_AutoShootV1`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OLD_AutoShootV1.md) — vision-driven AprilTag-aligned autonomous shooter (FSM diagram + state-by-state notes).

The team's current robot (as wired in `OLD_BasicOpMode_Linear` and `OLD_AutoSquare_Linear`) uses these `hardwareMap` names — keep them stable across new OpModes so they line up with the on-device config:

- **Drive motors** (4-wheel mecanum): `"leftFront"`, `"rightFront"`, `"leftBack"`, `"rightBack"`
  - `rightBack` must be set to `DcMotor.Direction.REVERSE` after lookup; the other three stay default. This compensates for how the motor is physically mounted — don't "fix" it by removing the reverse, and don't reverse the others.
- **Shooter**: `"flywheel"` (DcMotor)
- **Intake servos** (continuous-rotation): `"leftServo"`, `"rightServo"` (CRServo). They run as a pair — `leftServo` at `-1`, `rightServo` at `+1` together — because they're mirrored physically.
- **Odometry**: `"odo"` (`GoBildaPinpointDriver`). Tuned constants — copy these verbatim into any new OpMode that uses odometry rather than re-deriving them:
  - `setOffsets(-84.0, -168.0, DistanceUnit.MM)`
  - `setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD)`
  - `setEncoderDirections(REVERSED, FORWARD)` (X reversed, Y forward)
  - Call `resetPosAndIMU()` in init; call `odo.update()` once per loop iteration before reading position/heading.

### Mecanum kinematics
Use the same formula in both teleop and auto so behavior matches between modes. Inputs are `drive` (forward), `strafe` (right), `turn` (rotate):

```
frontLeft  = drive + strafe + turn
frontRight = drive - strafe - turn
backLeft   = drive - strafe + turn
backRight  = drive + strafe - turn
```

Then normalize by the max absolute value if any wheel exceeds 1.0. The `rightBack` `REVERSE` direction is what makes a positive `turn` value rotate the robot consistently with the other wheels — don't try to also flip the sign in the formula.

### OpMode naming
Driver-Station names end with `ChargedCreeper` (e.g. `"Basic: Linear OpMode ChargedCreeper"`, `"Auto: Square Loop ChargedCreeper"`) so the team's OpModes are easy to spot in the menu. Use `group="Linear OpMode"` for both teleop and the team's autos.

### Style note
`OLD_BasicOpMode_Linear` actually extends iterative `OpMode` (not `LinearOpMode`) despite its name — the file was forked from the SDK sample and renamed but kept the iterative structure. New autonomous code should extend `LinearOpMode` and use `runOpMode()` with `waitForStart()` + a `while (opModeIsActive())` loop; this is the style used by `OLD_AutoSquare_Linear`.

## Editing conventions specific to this SDK

- **Don't edit `FtcRobotController/external/samples/`** — they're reference material that gets updated when the SDK is rev'd. All team work happens in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- **Don't bump `build.dependencies.gradle` SDK versions** casually — those track FTC's annual SDK releases and need to stay in sync with the firmware on the Control Hub.
- **Don't edit `build.common.gradle`** unless absolutely necessary; team-specific build customization belongs in `TeamCode/build.gradle`.
- Source compatibility is Java 1.8. OnBotJava (the in-browser editor that some teams use alongside Android Studio) only supports 1.8 — anything that builds here should also be parseable by OnBotJava if teams want to edit it on the robot.
