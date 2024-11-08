package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.armcommands.ManualArmCommand;
import org.firstinspires.ftc.teamcode.commands.presets.ArmToScoringCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.CRDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    //arm stuff
    private Arm arm;
    private Slides lift;
    private CRDiffy diffy;
    private Claw claw;
    private Drivebase db;
//    private Limelight limelight;

    private ManualArmCommand manualArmCommand;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        arm = new Arm(hardwareMap);
        diffy = new CRDiffy(hardwareMap);
        lift = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        db = new Drivebase(hardwareMap);
//        limelight = new Limelight(hardwareMap, 5.0);
        telemetry.addData(">", "Hardware Map Initialized");
        telemetry.update();

        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Start the Limelight
//        limelight.startLimelight();
//        telemetry.addData(">", "Limelight Ready");
//        telemetry.update();

        manualArmCommand = new ManualArmCommand(arm, manipulator);

        arm.setDefaultCommand(new PerpetualCommand(manualArmCommand));

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.Y)) //Triangle
                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.BASKET_HIGH)
                        .withTimeout(2500)
                        .interruptOn(() -> manualArmCommand.isManualActive()));

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B))
                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.GRAB_SUB) //GRAB_SUB
                        .withTimeout(2500)
                        .interruptOn(() -> manualArmCommand.isManualActive()));

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.HOVER_SUB)
                        .interruptOn(() -> manualArmCommand.isManualActive()));

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.A))
                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.REST)
                        .withTimeout(2500)
                        .interruptOn(() -> manualArmCommand.isManualActive()));

        telemetry.addData(">", "Commands Ready");
        telemetry.update();

        telemetry.addData(">", "Robot Ready To Start");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        //Drivebase
        db.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //Reset
        if (gamepad1.square) {
            db.reset();
            gamepad1.rumble(250); // Angle recalibrated
        }

        //Arm
        if (gamepad2.right_stick_button) {
            arm.resetArmPosition();
        }
//        arm.setPower(gamepad2.right_stick_x);
        telemetry.addData("Arm Encoder Pos >", arm.getArmPosition());
        telemetry.addData("Interpolation>", arm.estimateArmPos());

        //Slides
        if (gamepad2.left_stick_button) {
            lift.resetLiftPosition();
        }
        lift.setLiftPower(gamepad2.left_stick_y);

        //Diffy
        while (gamepad2.dpad_left) {
            diffy.moveDiffy(0.4);
        } while (gamepad2.dpad_right) {
            diffy.moveDiffy(-0.4);
        }
        diffy.rotateDiffy(gamepad2.left_trigger - gamepad2.right_trigger);
        telemetry.addData("Left Axon", diffy.getLeftDiffyPose());
        telemetry.addData("Right Axon", diffy.getRightDiffyPose());

        //Claw
        if (gamepad2.left_bumper) {
            claw.switchState();
        }
        telemetry.addData("Is Open? >", claw.isOpen());

//        LLResult result = limelight.getLatestResult();
//
//        if (result != null && result.isValid()) {
//            double tx = result.getTx();  // Horizontal offset
//            double ty = result.getTy();  // Vertical offset
//
//            telemetry.addData("tx", tx);
//            telemetry.addData("ty", ty);
//
//            if (limelight.isTargetAligned(result)) {
//                telemetry.addData("Alignment", "Target is centered!");
//            } else {
//                telemetry.addData("Alignment", "Target is NOT centered!");
//            }
//        } else {
//            telemetry.addData("Limelight", "No valid data");
//        }
//        // Stop Limelight
//        limelight.stopLimelight();

        telemetry.addData("Lift Height", lift.getLiftPosition());
        telemetry.addData("Arm Pos", arm.getArmPosition());
        telemetry.addData("imuHeading", db.getCorrectedYaw());
        telemetry.addData("imuNONCO", db.imu.getYaw());
        telemetry.update();
    }
}
