package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Dashboard Test ChargedCreeper", group = "Linear OpMode")
public class DashboardTest extends OpMode {
    public static double FLYWHEEL_VELOCITY = 2600;

    private DcMotorEx flywheel;
    private CRServo leftServo;
    private CRServo rightServo;
    private boolean servoIsRunning;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        servoIsRunning = false;
    }

    @Override
    public void loop() {
        flywheel.setVelocity(FLYWHEEL_VELOCITY);
        telemetry.addData("Target Velocity", FLYWHEEL_VELOCITY);
        telemetry.addData("Actual Velocity", flywheel.getVelocity());
        telemetry.update();
        if (gamepad1.a) {
            servoIsRunning = true;
        } else if (gamepad1.b) {
            servoIsRunning = false;
        }
        if (servoIsRunning) {
            leftServo.setPower(-1);
            rightServo.setPower(1);

        } else {
            leftServo.setPower(0);
            rightServo.setPower(0);
            flywheel.setVelocity(0);
        }
    }
}
