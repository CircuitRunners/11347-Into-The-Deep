package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class SlidePIDTestKooky extends OpMode {
    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.0001; // 0.022, 0, 0.001
    public static double f = 0; // 0.15

    public static int target = 0;

    public static double powerMultiplier = 1.0; // 0.5

    private final double ticks_in_degree = 700 / 100.0;

    DcMotorEx leftSlideMotor;
    DcMotorEx rightSlideMotor;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "left");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "right");
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Negate the gravity when stopped ?
        //TODO gravity PID coefficients?
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int slidePos = rightSlideMotor.getCurrentPosition();
       double pid = controller.calculate(slidePos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        rightSlideMotor.setPower(power * powerMultiplier);
        leftSlideMotor.setPower(power * powerMultiplier);

        telemetry.addData("pos>", slidePos);
        telemetry.addData("target>", target);
        telemetry.update();
    }
}
