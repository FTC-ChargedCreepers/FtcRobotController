package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(8.3)
                    .forwardZeroPowerAcceleration(-30.76994149927475)
                    .lateralZeroPowerAcceleration(-49.244087541016555)
                    .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.00125, 0.024))
                    .headingPIDFCoefficients(new PIDFCoefficients(0.64, 0, 0.05, 0.03));
    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("rightFront")
                    .rightRearMotorName("rightBack")
                    .leftRearMotorName("leftBack")
                    .leftFrontMotorName("leftFront")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .xVelocity(66.73178364911418)
                    .yVelocity(56.74286513441191);
    // need velocity data from gamepad
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants localizerConstants =
            new PinpointConstants()
                    // distance needs to be changed later
                    .forwardPodY(4.6259845)
                    .strafePodX(8.1692913)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName("odo")
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // HEHSUTF THE HELL UP PEDRO YOU ARE NOT FUNNY STOP MAKING FUN OF ME I AM A HUMAN BEING WITH
    // FEELINGS AND YOU ARE A ROBOT WHO CAN'T FEEL ANYTHING SO STOP MAKING FUN OF ME PLEASE I AM
    // BEGGING YOU STOP IT NOW PLEASE
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
// D:
