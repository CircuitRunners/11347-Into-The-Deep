package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name="Arm Linear Interpolation Value Tester", group="0")
@Config
public class Test extends CommandOpMode {

//    DcMotorEx armMotor;
    ElapsedTime timer;
    public static double diff = 0;
    Arm arm;
    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

//        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        timer = new ElapsedTime();
        arm = new Arm(hardwareMap);

//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void run() {
        super.run();
        if (timer.milliseconds() >= 500) { // originally 700, need to test with 500
            if (gamepad2.left_bumper) {
                diff -= 0.01;
            } else if (gamepad2.right_bumper) {
                diff += 0.01;
            }
            timer.reset();
        }

        arm.setPowerTesting(gamepad2.right_stick_x);

//        armMotor.setPower(gamepad2.right_stick_x + diff);
        telemetry.addData("Diff Value>", diff);
        telemetry.addData("Arm Encoder>", arm.getArmPosition());
        telemetry.addData("Interpolation>", arm.estimateArmPos());
        telemetry.update();
    }
}
