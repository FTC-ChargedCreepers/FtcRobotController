package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto: Square Loop ChargedCreeper", group = "Linear OpMode")
@Disabled
public class AutoSquare_Linear extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private GoBildaPinpointDriver odo;

    private static final double DRIVE_POWER = 0.5;
    private static final double TURN_POWER = 0.4;
    private static final double TURN_MIN_POWER = 0.15;
    private static final double SIDE_SECONDS = 1.5;
    private static final double TURN_TARGET_DEG = 90.0;
    private static final double TURN_TOLERANCE_DEG = 1.5;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight.setDirection(DcMotor.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int lap = 0;
        while (opModeIsActive()) {
            lap++;
            for (int side = 1; side <= 4 && opModeIsActive(); side++) {
                telemetry.addData("Lap", lap);
                telemetry.addData("Side", side);
                telemetry.update();

                driveForward(SIDE_SECONDS);
                if (!opModeIsActive()) break;
                turnByHeading(TURN_TARGET_DEG);
            }
        }
    }

    private void driveForward(double seconds) {
        setMecanum(DRIVE_POWER, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            odo.update();
            idle();
        }
        stopMotors();
    }

    // Turn until the heading has changed by `targetDegrees` (absolute), using
    // odometry IMU heading. Direction follows the sign convention of the mecanum
    // turn input — flip TURN_POWER's sign if the robot rotates the wrong way.
    private void turnByHeading(double targetDegrees) {
        odo.update();
        double startHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            odo.update();
            double current = odo.getPosition().getHeading(AngleUnit.DEGREES);
            double turned = Math.abs(normalizeAngle(current - startHeading));
            double remaining = targetDegrees - turned;

            if (remaining <= TURN_TOLERANCE_DEG) break;

            double power = Math.max(TURN_MIN_POWER, Math.min(TURN_POWER, remaining * 0.02));
            setMecanum(0, 0, power);

            telemetry.addData("Heading", "%.2f", current);
            telemetry.addData("Turned", "%.2f", turned);
            telemetry.addData("Remaining", "%.2f", remaining);
            telemetry.update();
        }
        stopMotors();
    }

    private static double normalizeAngle(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private void setMecanum(double drive, double strafe, double turn) {
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

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
