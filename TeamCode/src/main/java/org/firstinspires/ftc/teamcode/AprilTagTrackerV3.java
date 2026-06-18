package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "AprilTag Tracker V3")

public class AprilTagTrackerV3 extends OpMode {

    private enum State {
        FIND_TAG,
        CENTER_TAG,
        BUILD_PATH,
        FOLLOW_PATH,
        DONE
    }

    private State state = State.FIND_TAG;

    private Follower follower;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    private static final int DESIRED_TAG_ID = 24;
    private static final boolean USE_WEBCAM = true;

    private boolean exposureConfigured = false;
    private int centeringFrames = 0;

    private double fieldX;
    private double fieldY;
    private double robotHeading;

    private Path path;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized");
    }

    @Override
    public void init_loop() {
        if (!exposureConfigured &&
                visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            setManualExposure(6, 50);
            exposureConfigured = true;
        }

        telemetry.addLine("Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        state = State.FIND_TAG;
    }

    @Override
    public void loop() {
        switch (state) {
            case FIND_TAG:
                desiredTag = findDesiredTag();
                if (desiredTag == null) {
                    moveRobot(0, 0, 0.2);
                    telemetry.addLine("Scanning...");
                } else {
                    stopDrive();
                    centeringFrames = 0;
                    state = State.CENTER_TAG;
                }
                break;
            case CENTER_TAG:
                desiredTag = findDesiredTag();

                if (desiredTag != null) {
                    double offset = desiredTag.ftcPose.x;
                    if (Math.abs(offset) > 2.0) {
                        centeringFrames = 0;
                        double turn = Math.max(-0.25, Math.min(0.25, offset * 0.03));
                        moveRobot(0, 0, turn);
                    } else {
                        stopDrive();
                        centeringFrames++;
                        if (centeringFrames > 10) {
                            calculateRobotPose();
                            state = State.BUILD_PATH;
                        }
                    }
                }else {
                    stopDrive();
                }

                break;
            case BUILD_PATH:
                follower.setStartingPose(
                        new Pose(fieldX, fieldY, robotHeading));

                path = new Path(
                        new BezierLine(
                                new Pose(fieldX, fieldY),
                                new Pose(70.75, 80)
                        ));

                path.setLinearHeadingInterpolation(
                        robotHeading,
                        Math.toRadians(40));

                follower.followPath(path);

                state = State.FOLLOW_PATH;
                break;

            case FOLLOW_PATH:

                follower.update();

                if (!follower.isBusy()) {
                    stopDrive();
                    state = State.DONE;
                }

                break;

            case DONE:
                stopDrive();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    public void moveRobot(double drive, double strafe, double turn) {
        double[] speeds = {
                (drive + strafe + turn),
                (drive - strafe - turn),
                (drive - strafe + turn),
                (drive + strafe - turn)
        };

        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        frontLeftDrive.setPower(speeds[0]);
        frontRightDrive.setPower(speeds[1]);
        backLeftDrive.setPower(speeds[2]);
        backRightDrive.setPower(speeds[3]);
    }

    private void initAprilTag() {
        AprilTagLibrary testLibrary = new AprilTagLibrary.Builder()
                .addTag(6, "FIH", 6.8125, DistanceUnit.INCH)
                .addTag(24, "RED", 6.5, DistanceUnit.INCH)
                .addTag(20, "BLUE", 6.5, DistanceUnit.INCH)
                .build();

        aprilTag = new AprilTagProcessor.Builder()
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

    /*
      Asynchronous safe manual exposure config for standard OpMode frameworks
    */
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

    private AprilTagDetection findDesiredTag() {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            if (detection.metadata == null) {
                continue;
            }

            if (detection.id == DESIRED_TAG_ID) {
                return detection;
            }
        }

        return null;
    }

    private void stopDrive() {
        moveRobot(0, 0, 0);
    }

    private void calculateRobotPose() {

        double yaw = Math.toRadians(-desiredTag.ftcPose.yaw);

        robotHeading = Math.toRadians(37) - yaw;

        double horizontal = desiredTag.ftcPose.range * (Math.sin(yaw + Math.toRadians(53)));
        double vertical = desiredTag.ftcPose.range * (Math.cos(yaw + Math.toRadians(53)));
        double tagFieldX = 129.1227;
        double tagFieldY = 126.3925;

        fieldX = tagFieldX - horizontal;
        fieldY = tagFieldY - vertical;

    }
}
