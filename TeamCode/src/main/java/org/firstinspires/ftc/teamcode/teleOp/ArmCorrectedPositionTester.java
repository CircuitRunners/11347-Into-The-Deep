package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected;

@TeleOp
@Config
public class ArmCorrectedPositionTester extends OpMode {
    public static int target = 0;
    ArmCorrected arm;

    @Override
    public void init() {
        arm = new ArmCorrected(hardwareMap, telemetry);
        arm.init();
        arm.start();
        telemetry.addData("Arm Pos>", arm.getArmPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        arm.setTarget(target);
        arm.updatePIDF();
        telemetry.addData("Arm Pos>", arm.getArmPosition());
        telemetry.update();
    }
}
