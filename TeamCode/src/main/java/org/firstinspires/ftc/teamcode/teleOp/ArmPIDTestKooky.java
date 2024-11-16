package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ArmPIDTestKooky extends OpMode {
    private PIDController controller;

    public static double p = 0.022, i = 0, d = 0.001; // 0.022, 0, 0.001
    public static double f = 0.15; // 0.15

    public static int target = 0;

    public static double powerMultiplier = 1.0; // 0.5

    private final double ticks_in_degree = 700 / 100.0;

    private DcMotorEx arm_motor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "armMotor");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power * powerMultiplier);

        telemetry.addData("pos>", armPos);
        telemetry.addData("target>", target);
        telemetry.update();
    }
}
