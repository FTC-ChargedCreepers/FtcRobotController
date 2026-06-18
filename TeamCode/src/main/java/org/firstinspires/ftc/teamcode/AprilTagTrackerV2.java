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

import static org.firstinspires.ftc.teamcode.Tuning.stopRobot;


@Autonomous(name = "AprilTag Tracker V2")

public class AprilTagTrackerV2 extends OpMode {

    private Follower follower;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 24;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private final ArrayList<Double> velocities = new ArrayList<>();
    private boolean end;

    // Variable to ensure exposure optimization only attempts setup once
    private boolean exposureConfigured = false;

    private Path forwards;

    // Global variables to lock down coordinates
    private double fieldX = 0;
    private double fieldY = 0;
    private double robotHeading = 0;
    private boolean tagFoundInInit = false;

    private int centeringFramesCount = 0; // Ensures it holds center for a moment


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        initAprilTag();

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rightBack");

        // Single motor reverse layout adjusted to your chassis configuration
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        exposureConfigured = false; // Reset setup flag

        telemetry.addData("Status", "Hardware Initialized safely.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Handle camera exposure warmup
        if (USE_WEBCAM && !exposureConfigured) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                setManualExposure(6, 50);
                exposureConfigured = true;
            } else {
                telemetry.addLine("Camera Status: Warming up stream pipeline...");
                telemetry.update();
                return;
            }
        }

        boolean targetFound = false;
        desiredTag = null;

        // Fetch current visibility frame
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                targetFound = true;
                desiredTag = detection;
                break;
            }
        }

        // CRITICAL BUG FIX: Only track layout configurations if a target physically exists
        if (targetFound) {
            double currentXOffset = desiredTag.ftcPose.x;

            if (Math.abs(currentXOffset) > 2) {
                telemetry.addLine("Target Found. Attempting to Center in Frame...");
                telemetry.addData("Current Frame Offset", "%3.2f inches", currentXOffset);

                // Safe tuning loop layout adjusted for the upside-down webcam placement
                double turnGain = 0.03;
                centeringFramesCount = 0; // Reset the stability counter

                double turnPower = currentXOffset * turnGain;

                // Clip power to keep movements predictable and slow
                turnPower = Math.max(-0.25, Math.min(0.25, turnPower));

                //follower.turn();

                // Pivot on the spot toward the tag
                moveRobot(0, 0, turnPower);

            } else {
                // The tag is centered! Stop the wheels and let it stabilize over a few frames
                moveRobot(0, 0, 0);
                centeringFramesCount++;

                if (centeringFramesCount > 10) { // Stable for 10 frames straight
                    telemetry.addLine("\n> Target Found and Locked");

                    // Corrects the yaw measurement to account for the upside-down camera orientation
                    double yaw = Math.toRadians(-desiredTag.ftcPose.yaw);

                    robotHeading = Math.toRadians(37) - yaw;

                    double horizontal = desiredTag.ftcPose.range * (Math.sin(yaw + Math.toRadians(53)));
                    double vertical = desiredTag.ftcPose.range * (Math.cos(yaw + Math.toRadians(53)));
                    double tagFieldX = 129.1227;
                    double tagFieldY = 126.3925;

                    fieldX = tagFieldX - horizontal;
                    fieldY = tagFieldY - vertical;

                    tagFoundInInit = true;

                    telemetry.addData("Calculated X", "%3.2f", fieldX);
                    telemetry.addData("Calculated Y", "%3.2f", fieldY);
                    telemetry.addData("Calculated Heading", "%3.2f°", Math.toDegrees(robotHeading));
                }
            }
        } else {
            // Target lost out of frame
            moveRobot(0, 0, 0.2);
            centeringFramesCount = 0;
            telemetry.addLine("\n> Scanning...");
        }

        telemetry.update();
    }

    public void start() {
        follower.activateAllPIDFs();

        if (tagFoundInInit) {
            follower.setStartingPose(new Pose(fieldX, fieldY, robotHeading));
            forwards = new Path(new BezierLine(new Pose(fieldX, fieldY), new Pose(70.75, 80)));
            forwards.setLinearHeadingInterpolation(robotHeading, Math.toRadians(40));
        }

        if (forwards != null) {
            follower.followPath(forwards);
        } else {
            telemetry.addLine("No AprilTag was found before START was pressed");
        }

    }

    public void loop() {
        // Keep Pedro Pathing's internal localizer and follower algorithms running
        follower.update();

        // Check if the robot is currently busy driving the Bezier path
        if (follower.isBusy()) {
            telemetry.addLine("Robot is currently following the path to target...");
        } else {
            // The path has finished executing naturally!
            if (!end) {
                end = true;
                 moveRobot(0,0,0);
            }
            telemetry.addLine("Path Execution Complete");
        }

        // Print real-time status updates directly to your Driver Station
        telemetry.addData("Current X Position", follower.getPose().getX());
        telemetry.addData("Current Y Position", follower.getPose().getY());
        //telemetry.addData("AprilTag Range", desiredTag.ftcPose.range);
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
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}