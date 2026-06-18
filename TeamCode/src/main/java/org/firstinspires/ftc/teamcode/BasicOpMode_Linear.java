/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Range;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Basic: Linear OpMode ChargedCreeper", group = "Linear OpMode")
public class BasicOpMode_Linear extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private DcMotorEx flywheel;
    public CRServo leftServo, rightServo;
    private boolean servoIsRunning;
    private GoBildaPinpointDriver odo;
    private double targetHeading = 0;
    private Pose2D previousPos;
    private boolean exposureConfigured = false;

    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;
    private VisionPortal visionPortal;

    private double DESIRED_TAG_ID = 24;
    private Follower follower;

    private static final double getTriangleAngle(double oppositeLeg, double adjacentLeg) {
        return Math.toDegrees(Math.atan2(oppositeLeg, adjacentLeg));
    }

    private static double getVelocity(double goalDist){
        return MathFunctions.clamp(0.0470181*Math.pow(goalDist, 2)- 0.083729*goalDist +1233.60264, 0 , 2000);
    }

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        follower = Constants.createFollower(hardwareMap);
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        servoIsRunning = false;
        odo.setOffsets(
                -84.0,
                -168.0,
                DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
        odo.setPosX(72, DistanceUnit.INCH);
        odo.setPosY(14.5, DistanceUnit.INCH);

        initAprilTag();
    }

    private void initAprilTag() {
        AprilTagLibrary testLibrary =
                new AprilTagLibrary.Builder()
                        .addTag(6, "FIH", 6.8125, DistanceUnit.INCH)
                        .addTag(24, "RED", 6.5, DistanceUnit.INCH)
                        .addTag(20, "BLUE", 6.5, DistanceUnit.INCH)
                        .build();

        aprilTag =
                new AprilTagProcessor.Builder()
                        .setDrawTagOutline(true)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setTagLibrary(testLibrary)
                        .setLensIntrinsics(243.0, 243.0, 320.0, 240.0)
                        .build();

        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "NeverGonnaGiveYouUp"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double[] speeds = {
            (drive + strafe + turn),
            (drive - strafe - turn),
            (drive - strafe + turn),
            (drive + strafe - turn)
        };

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }

        // If and only if the maximum is outside the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

        if (gamepad1.a) {
            servoIsRunning = true;
        } else if (gamepad1.b) {
            servoIsRunning = false;
        }
        if (gamepad1.x){
            follower.turnTo(Math.toRadians(getTriangleAngle(
                    130-odo.getPosX(DistanceUnit.INCH),
                    130-odo.getPosY(DistanceUnit.INCH))));
        }
        desiredTag = findDesiredTag();

        if (servoIsRunning) {
            leftServo.setPower(-1);
            rightServo.setPower(1);
            flywheel.setVelocity(getVelocity(
                    Math.sqrt(Math.pow(130-odo.getPosX(DistanceUnit.INCH), 2) + Math.pow(130-odo.getPosY(DistanceUnit.INCH), 2))));
        } else {
            leftServo.setPower(0);
            rightServo.setPower(0);
            flywheel.setVelocity(0);
        }

        odo.update();
        Pose2D pos = odo.getPosition();
        if (servoIsRunning) {
            if (Math.abs(turn) > 0.1) {
                // Got turn input. The robot is turning. Update the target heading.
                targetHeading = pos.getHeading(AngleUnit.DEGREES);
            } else {
                // the robot is deviating from its targeted direction.
                // Try to ensure it continues the targeted direction.
                driveStraight(pos);
            }
        }

        telemetry.addData("ODO Status", odo.getDeviceStatus());
        telemetry.addData(
                "ODO Frequency",
                odo.getFrequency()); // prints/gets the current refresh rate of the PinpointS

        String data =
                String.format(
                        Locale.US,
                        "{X: %.3f, Y: %.3f, H: %.3f}",
                        pos.getX(DistanceUnit.MM),
                        pos.getY(DistanceUnit.MM),
                        pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        String velocity =
                String.format(
                        Locale.US,
                        "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                        odo.getVelX(DistanceUnit.MM),
                        odo.getVelY(DistanceUnit.MM),
                        odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        // Show the elapsed game time and wheel power
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData(
                "Motors",
                "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)",
                speeds[0],
                speeds[1],
                speeds[2],
                speeds[3]);

        telemetry.addData("ServoIsRunning", servoIsRunning);
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());

        telemetryAprilTag();

        telemetry.update();

    }

    private AprilTagDetection findDesiredTag() {
        List <AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == DESIRED_TAG_ID) {
                return detection;
            }
        }
        return null;
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(
                        String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(
                        String.format(
                                "XYZ %6.1f %6.1f %6.1f  (inch)",
                                detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(
                        String.format(
                                "PRY %6.1f %6.1f %6.1f  (deg)",
                                detection.ftcPose.pitch,
                                detection.ftcPose.roll,
                                detection.ftcPose.yaw));
                telemetry.addLine(
                        String.format(
                                "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                                detection.ftcPose.range,
                                detection.ftcPose.bearing,
                                detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(
                        String.format(
                                "Center %6.0f %6.0f   (pixels)",
                                detection.center.x, detection.center.y));
            }
        } // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    } // end method telemetryAprilTag()

    private void driveStraight(Pose2D pos) {
        if (previousPos != null) {
            double distanceDriven = pos.getY(DistanceUnit.MM) - previousPos.getY(DistanceUnit.MM);
            if (distanceDriven > 10) {
                double headingError = targetHeading - pos.getHeading(AngleUnit.DEGREES);
                double kP = 0.02;
                double correction = headingError * kP;
                double drivePower = 0.5;
                double leftPower = drivePower + correction;
                double rightPower = drivePower - correction;
                leftServo.setPower(leftPower);
                rightServo.setPower(rightPower);
            }
        }

        previousPos = pos;
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    @Override
    public void init_loop() {
        if (!exposureConfigured
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            setManualExposure(6, 50);
            exposureConfigured = true;
        }

        telemetry.addLine("Ready to start");
        telemetry.update();
    }

}
