package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrectedTwoPointOh;
import org.firstinspires.ftc.teamcode.subsystems.CRDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp
public class MainTeleOpTest extends CommandOpMode {
    // Arm and other subsystems
    private ArmCorrectedTwoPointOh arm;
    private Slides lift;
    private CRDiffy diffy;
    private Claw claw;
    private Drivebase db;

    @Override
    public void initialize() {
        // Initialize hardware and commands
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        arm = new ArmCorrectedTwoPointOh(hardwareMap);
        diffy = new CRDiffy(hardwareMap);
        lift = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        db = new Drivebase(hardwareMap);

        // Initialize the claw in a closed state
        claw.close();
        telemetry.addData(">", "Hardware Map Initialized");
        telemetry.update();

        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Default telemetry
        telemetry.addData(">", "Robot Ready To Start");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // Drivebase control
        db.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Reset the drivebase heading
        if (gamepad1.square) {
            db.reset();
            gamepad1.rumble(250); // Notify driver with rumble
        }

        // Arm control
        if (gamepad2.right_stick_button) {
            arm.resetArmPosition();
        }

        // Move arm to specific preset positions using DPAD
        if (gamepad2.dpad_up) {
            arm.toTopBar(); // Move to top bar position
        } else if (gamepad2.dpad_down) {
            arm.toRestPos(); // Move to rest position
        } else if (gamepad2.dpad_left) {
            arm.toGrabPos(); // Move to grab position
        } else if (gamepad2.dpad_right) {
            arm.toBasketPos(); // Move to basket position
        }

        // Manual arm adjustment using the right stick Y-axis
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            arm.manual((int) (gamepad2.right_stick_y * 10)); // Fine-tune arm position
        }

        // Slides control
        if (gamepad2.left_stick_button) {
            lift.resetLiftPosition();
        }

        // Diffy control
//        if (gamepad2.dpad_left) {
//            diffy.moveDiffy(0.4);
//        } else if (gamepad2.dpad_right) {
//            diffy.moveDiffy(-0.4);
//        }
        diffy.rotateDiffy(gamepad2.left_trigger - gamepad2.right_trigger);

        // Claw control
        if (gamepad2.left_bumper) {
            claw.switchState();
        }

        // Display telemetry
        telemetry.addData("Lift Height", lift.getLiftPosition());
        telemetry.addData("Arm Pos", arm.getCurrentPosition());
        telemetry.addData("Target Pos", arm.getArmTarget());
        telemetry.addData("IMU Heading", db.getCorrectedYaw());
        telemetry.addData("Is Claw Open?", claw.isOpen());
        telemetry.update();
    }
}
