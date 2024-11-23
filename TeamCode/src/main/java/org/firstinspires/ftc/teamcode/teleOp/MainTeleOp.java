package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.armcommands.ManualArmCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.presets.ArmToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.LiftToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.testDownCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.CRDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    //arm stuff
    private Arm arm;
    private Slides lift;
    private Diffy diffy;
    private Claw claw;
    private Drivebase db;
//    private Limelight limelight;

    private ManualLiftCommand manualLiftCommand;
    private ManualArmCommand manualArmCommand;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        arm = new Arm(hardwareMap);
        diffy = new Diffy(hardwareMap);
        lift = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        db = new Drivebase(hardwareMap);
//        limelight = new Limelight(hardwareMap, 5.0);
        claw.closeClaw();
        telemetry.addData(">", "Hardware Map Initialized");
        telemetry.update();

        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Start the Limelight
//        limelight.startLimelight();
//        telemetry.addData(">", "Limelight Ready");
//        telemetry.update();

        // Command Stuff
        manualLiftCommand = new ManualLiftCommand(lift, manipulator);
        manualArmCommand = new ManualArmCommand(arm, manipulator);

        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));
        arm.setDefaultCommand(new PerpetualCommand(manualArmCommand));

        //HIGH BASKET PRESETS
        //LIFT
//        new Trigger(() -> manipulator.getLeftY() > 0.4)
//                .whenActive(new LiftToScoringCommand(lift, LiftToScoringCommand.Presets.HIGH)
//                        .withTimeout(3500)
//                        .interruptOn(() -> manualLiftCommand.isManualActive()));
//        //ARM
//        new Trigger(() -> manipulator.getLeftY() > 0.4)
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.BASKET_HIGH)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//
//        //REST PRESETS
//        new Trigger(() -> manipulator.getLeftY() < -0.4)
//                .whenActive(new testDownCommand(lift, arm, claw)
//                        .withTimeout(3500)
//                        .interruptOn(() -> manualArmCommand.isManualActive() || manualLiftCommand.isManualActive()));
//
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.RIGHT_BUMPER))
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.SPECIMEN)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//
//
//        //ARM
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.Y)) // Triangle
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.HOVER_SUB)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//        //ARM
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B)) // Circle
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.GRAB_SUB) //GRAB_SUB
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//        //ARM
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.A)) // Cross
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.BASKET_HIGH)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//        //ARM
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.X)) // Square
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.REST)
//                        .withTimeout(2500)
//                                .interruptOn(() -> manualArmCommand.isManualActive()));

        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B)) // Circle
                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.TOPBAR)
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
//        lift.setLiftPower(gamepad2.left_stick_y);

        //Diffy
//        while (gamepad2.dpad_left) {
//            diffy.moveDiffy(0.4);
//        } while (gamepad2.dpad_right) {
//            diffy.moveDiffy(-0.4);
//        }
//        diffy.rotateDiffy(gamepad2.left_trigger - gamepad2.right_trigger);
//        telemetry.addData("Left Axon", diffy.getLeftDiffyPose());
//        telemetry.addData("Right Axon", diffy.getRightDiffyPose());

        if (gamepad2.right_bumper) {
            diffy.CenterDiffy();
        }

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
