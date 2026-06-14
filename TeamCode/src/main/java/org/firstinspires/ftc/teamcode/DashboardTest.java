package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Dashboard Test ChargedCreeper", group = "Linear OpMode")
public class DashboardTest extends OpMode {
    public static double FLYWHEEL_VELOCITY = 2600;

    private DcMotorEx flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        flywheel.setVelocity(FLYWHEEL_VELOCITY);
        telemetry.addData("Target Velocity", FLYWHEEL_VELOCITY);
        telemetry.addData("Actual Velocity", flywheel.getVelocity());
        telemetry.update();
    }
}
