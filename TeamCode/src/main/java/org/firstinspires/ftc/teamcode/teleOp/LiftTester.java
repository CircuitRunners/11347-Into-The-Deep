package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.presets.LiftToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.testCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp (name="Lift Tester")
public class LiftTester extends CommandOpMode {
    private Slides lift;
    private Arm arm;
    private Claw claw;
    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;

    @Override
    public void initialize(){
        // Use a bulk cache to loop faster using old values instead of blocking a thread kinda
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        lift = new Slides(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);

        manualLiftCommand = new ManualLiftCommand(lift, manipulator);
        manualLiftResetCommand = new ManualLiftResetCommand(lift, manipulator);

        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

        new Trigger(() -> manipulator.getLeftY() > 0.4)
                .whenActive(new LiftToScoringCommand(lift, LiftToScoringCommand.Presets.HIGH)
                        .withTimeout(3500)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        new Trigger(() -> manipulator.getLeftY() < -0.4)
                .whenActive(new LiftToScoringCommand(lift, LiftToScoringCommand.Presets.DOWN)
                        .withTimeout(2500)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Mid preset
        new Trigger(() -> manipulator.getRightY() > -0.4)
                .whenActive(new LiftToScoringCommand(lift, LiftToScoringCommand.Presets.SHORT)
                        .withTimeout(2500)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Short preset
        new Trigger(() -> manipulator.getRightY() < 0.4)
                .whenActive(new LiftToScoringCommand(lift, LiftToScoringCommand.Presets.MID)
                        .withTimeout(3000)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.A))
                .whenActive(new testCommand(lift, arm, claw)
                        .withTimeout(3500)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        super.run();

        telemetry.addData("position", lift.getLiftPosition());

//        if (gamepad1.left_bumper) {
//            v4b.setPosition(V4B.V4BState.RETRACT);
//        } else if(gamepad1.right_bumper) {
//            v4b.setPosition(V4B.V4BState.EXTEND);
//        }
//
//        if (gamepad2.right_bumper) {
//            outtake.transport();
//        } else if (gamepad2.left_bumper) {
//            outtake.open();
//        }

//        arm.setPowerActual(0.5);

        telemetry.addData("position >", arm.getArmPosition());
        telemetry.update();
    }
}