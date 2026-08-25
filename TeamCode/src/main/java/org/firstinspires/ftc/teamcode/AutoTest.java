package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "8lb Fish Auto Test")
public class AutoTest extends OpMode {

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private DcMotorEx intake;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight3A;


    @Override
    public void init() {

        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9); //current color tracking pipeline

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
//        odo.setOffsets(
//                -84.0,
//                -168.0,
//                DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.REVERSED,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
//        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
//        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
    }

    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {

        odo.update();
        Pose2D pos = odo.getPosition();

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y offset", llResult.getTy());
            telemetry.addData("Target Area percent", llResult.getTa());
        }

        telemetry.update();
    }

}
