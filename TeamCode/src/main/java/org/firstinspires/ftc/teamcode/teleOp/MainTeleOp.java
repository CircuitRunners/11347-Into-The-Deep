package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.support.Actions;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.armcommands.ManualArmCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand;
import org.firstinspires.ftc.teamcode.commands.presets.ArmToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.LiftToScoringCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.commands.presets.LiftToRestingCommand;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    //arm stuff
    private ArmCorrected arm;
    private Slides lift;
    private Diffy diffy;
    private Claw claw;
    private Diffy.ServoStates currentState;
    private Drivebase db;
//    private Limelight limelight;

    private ManualLiftCommand manualLiftCommand;
    private ManualArmCommand manualArmCommand;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        arm = new ArmCorrected(hardwareMap);
        diffy = new Diffy(hardwareMap);
        currentState = Diffy.ServoStates.START;
        lift = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        db = new Drivebase(hardwareMap);
//        limelight = new Limelight(hardwareMap, 5.0);
//        claw.close();
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
        manipulator.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(new LiftToScoringCommand(lift, LiftToScoringCommand.Presets.MID, arm)
                        .withTimeout(3500)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));
//        //ARM
//        new Trigger(() -> manipulator.getLeftY() > 0.4)
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.BASKET_HIGH)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));

        //REST PRESETS
//        new Trigger(() -> manipulator.getLeftY() < -0.4)
//                .whenActive(new testDownCommand(lift, arm, claw)
//                        .withTimeout(3500)
//                        .interruptOn(() -> manualArmCommand.isManualActive() || manualLiftCommand.isManualActive()));
//
//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B))
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.SPECIMEN)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
//
//
//        //ARM
        manipulator.getGamepadButton(GamepadKeys.Button.Y) // Triangle
                .whenActive(new ArmToScoringCommand(arm, claw, diffy, ArmToScoringCommand.Presets.HOVER_SUB)
                        .withTimeout(2500)
                        .interruptOn(() -> manualArmCommand.isManualActive()));

//        //ARM
//        new manipulator.getButton(GamepadKeys.Button.B)) // Circle
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.GRAB_SUB) //GRAB_SUB
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));
        //ARM
         manipulator.getGamepadButton(GamepadKeys.Button.B) // Circle
                .whenActive(new ArmToScoringCommand(arm, claw, diffy, ArmToScoringCommand.Presets.BASKET_HIGH)
                        .withTimeout(2500)
                        .interruptOn(() -> manualArmCommand.isManualActive()));
//        //ARM
//        manipulator.getGamepadButton(GamepadKeys.Button.X) // Square
//                .whenActive(new ArmToScoringCommand(arm, claw, diffy, ArmToScoringCommand.Presets.REST)
//                        .withTimeout(2500)
//                                .interruptOn(() -> manualArmCommand.isManualActive()));
        manipulator.getGamepadButton(GamepadKeys.Button.X) // Square
                .whenActive(new LiftToRestingCommand(lift, LiftToRestingCommand.Presets.DOWN, arm)
                        .withTimeout(2500)
                        .interruptOn(() -> manualArmCommand.isManualActive()));

//        new Trigger(() -> manipulator.getButton(GamepadKeys.Button.B)) // Circle
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.TOPBAR)
//                        .withTimeout(2500)
//                        .interruptOn(() -> manualArmCommand.isManualActive()));


        telemetry.addData(">", "Commands Ready");
        telemetry.update();

        telemetry.addData(">", "Robot Ready To Start");
        telemetry.update();
    }

    boolean previousButtonState = false;
    private boolean toggleDirectionForward = true;

    @Override
    public void run() {
        super.run();
        //Drivebase
        db.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        arm.update();
        //Reset
        if (gamepad1.square) {
            db.reset();
            gamepad1.rumble(250); // Angle recalibrated
        }

        //Arm
        if (gamepad2.square){
            Actions.runBlocking(arm.toTopBar);
            arm.toTopBar();
            arm.update();

        }
//        while (gamepad2.circle) {
//            diffy.centerDiffy();
//            currentState = Diffy.ServoStates.CENTER;
//            break;
//        }
      arm.manual(-gamepad2.right_stick_y);

        //Slides
        if (gamepad2.left_stick_button) {
            lift.resetLiftPosition();
        }
        lift.setLiftPower(gamepad2.left_stick_y);


        while (gamepad2.dpad_left) {
            diffy.rotateDiffyL();
        } while (gamepad2.dpad_right) {
            diffy.rotateDiffyR();
        }

        while (gamepad2.dpad_up) {
            diffy.moveDiffyP();
        }
        while (gamepad2.dpad_down) {
            diffy.moveDiffyN();
        }

        boolean currentButtonState = gamepad2.right_bumper;
        if (currentButtonState && !previousButtonState) {
            toggleDiffyPosition();
        }
        previousButtonState = currentButtonState;



        //Claw
        if (gamepad2.left_bumper) {
            claw.switchState();
        }

        telemetry.addData("Is Open? >", claw.isOpen());
        telemetry.addData("Lift Height", lift.getLiftPosition());
        telemetry.addData("Arm Pos", arm.getCurrentPosition());
        telemetry.addData("Arm Target", arm.getArmTarget());
        telemetry.addData("imuHeading", db.getCorrectedYaw());
        telemetry.addData("imuNONCO", db.imu.getYaw());
        telemetry.update();
    }

    public void toggleDiffyPosition() {
        if (toggleDirectionForward) {
            switch (currentState) {
                case START:
                    diffy.startDiffy();
                    currentState = Diffy.ServoStates.CENTER;
                    break;

                case CENTER:
                    diffy.centerDiffy();
                    currentState = Diffy.ServoStates.END;
                    break;

                case END:
                    diffy.endDiffy();
                    currentState = Diffy.ServoStates.CENTER;
                    toggleDirectionForward = false;
                    break;
            }
        } else {
            switch (currentState) {
                case CENTER:
                    diffy.startDiffy();
                    currentState = Diffy.ServoStates.START;
                    toggleDirectionForward = true;
                    break;
            }
        }

    }
}
