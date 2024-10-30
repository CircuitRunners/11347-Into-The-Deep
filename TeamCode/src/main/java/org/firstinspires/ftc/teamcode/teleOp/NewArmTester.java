package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.armcommands.ManualArmCommand;
import org.firstinspires.ftc.teamcode.commands.armcommands.ProfiledArmCommand;
import org.firstinspires.ftc.teamcode.commands.presets.ArmToScoringCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp
public class NewArmTester extends CommandOpMode {
    private Arm arm;
    private Claw claw;
    private ProfiledArmCommand pac;
    private ElapsedTime switchTimer;
    private ManualArmCommand manualArmCommand;
    double diff = 0;
    boolean isTrue = true;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx manipulator = new GamepadEx(gamepad2);

        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);

//        manualArmCommand = new ManualArmCommand(arm, manipulator);
//
//        arm.setDefaultCommand(new PerpetualCommand(manualArmCommand));
//
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_DOWN))
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.BASKET_HIGH)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_UP))
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.GRAB_SUB)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_LEFT))
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.HOVER_SUB)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_RIGHT))
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.REST)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));

        telemetry.addLine("Initialization Done");
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

//        //stops from changing gravity by more than 0.01 per press
//        if (switchTimer.milliseconds() >= 500) { // originally 700, need to test with 500
//            if (gamepad2.left_bumper) {
//                pac.gravity -= 0.01; // can be changed if its not enough
//            } else if (gamepad2.right_bumper) {
//                pac.gravity += 0.01;
//            }
//            switchTimer.reset();
//        }

        if (switchTimer.milliseconds() >= 500) {
            if (gamepad2.left_bumper) {
                isTrue = true;
            } else if (gamepad2.right_bumper) {
                isTrue = false;
            }
        }

        if (switchTimer.milliseconds() >= 500) {
            if (gamepad2.dpad_left) {
                diff -= 0.01;
            } else if (gamepad2.dpad_right) {
                diff += 0.01;
            }
        }

        arm.setPowerTesting(gamepad2.left_stick_y, diff, isTrue);

//        telemetry.addData("Gravity Coefficent >", pac.getGravity());
        telemetry.addData("Power Diff >", diff);
        telemetry.addData("isTrue? >", isTrue);
        telemetry.addData("position >", arm.getArmPosition());
        telemetry.update();
    }
}
