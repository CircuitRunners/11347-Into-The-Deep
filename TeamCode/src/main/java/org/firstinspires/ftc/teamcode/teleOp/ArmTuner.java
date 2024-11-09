package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
@TeleOp
public class ArmTuner extends OpMode {
    private PIDController controller;
    public static double kP = 0.0, kI = 0, kD = 0.0;
    public static double f = 0.0;
    public static double target = 0;
    private DcMotorEx arm_motor;
    private static double slowdown = 1.0;
    private static double offset = 0;

    @Override
    public void init() {
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        double armPos = (arm_motor.getCurrentPosition() - armStart) - offset;
        double pid = controller.calculate(armPos, target);

        double ff = Math.sin(Math.toRadians(target / TICKS_PER_RAD)) * f;

        double power = pid + ff;

        arm_motor.setPower(power * slowdown);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("power", arm_motor.getPower());
        telemetry.addData("Multiplier", slowdown);
        telemetry.update();
    }
}
