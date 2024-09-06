package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teleop2 extends CommandOpMode {
    public DcMotorEx FLM, FRM, BLM, BRM;
    public Gamepad g1, g2; // Unneeded if you are going to use gamepad1 or gamepad2 when getting inputs (ie: line 36, 37, 38)
    public IMU imu;

    IMU.Parameters imup;

    @Override
    public void initialize() {
        //initialize stuff
        FLM = hardwareMap.get(DcMotorEx.class, "FLM");
        FRM = hardwareMap.get(DcMotorEx.class, "FRM");
        BLM = hardwareMap.get(DcMotorEx.class, "BLM");
        BRM = hardwareMap.get(DcMotorEx.class, "BRM");
        g1 = gamepad1;
        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        imup = new IMU.Parameters(
                new RevHubOrientationOnRobot (
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(imup);

    }

    @Override
    public void run() {
        super.run();
        //this runs the motors
        double y = -gamepad1.left_stick_y; // If you are declaring a gamepad (ie: g1/g2), you can use that instead of gamepad1
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double bHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-bHeading) - y * Math.sin(-bHeading);
        double rotY = x * Math.sin(-bHeading) + y * Math.cos(-bHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flp = (rotY + rotX + rx) / denominator;
        double frp = (rotY - rotX - rx) / denominator;
        double blp = (rotY - rotX + rx) / denominator;
        double brp = (rotY + rotX - rx) / denominator;

        FLM.setPower(flp);
        FRM.setPower(frp);
        BLM.setPower(blp);
        BRM.setPower(brp);
    }
}
