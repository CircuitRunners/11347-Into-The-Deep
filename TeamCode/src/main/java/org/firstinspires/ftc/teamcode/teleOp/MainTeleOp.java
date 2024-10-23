package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.CRDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.SDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    //arm stuff
    private Arm arm;
    private Slides lift;
    private CRDiffy diffy;
    private Claw claw;
    private Drivebase db;
    private Limelight limelight;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        arm = new Arm(hardwareMap);
        diffy = new CRDiffy(hardwareMap);
        lift = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
        db = new Drivebase(hardwareMap);
        limelight = new Limelight(hardwareMap, 5.0);

        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Start the Limelight
        limelight.startLimelight();

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        //Arm
        arm.setPower(gamepad2.left_stick_x);

        //Slide
        lift.setLiftPower(gamepad2.left_stick_y);

        //Diffy
        diffy.moveDiffy(gamepad2.right_stick_y);
        diffy.rotateDiffy(gamepad2.right_trigger-gamepad2.left_trigger);
        telemetry.addData("Left Axon", diffy.getLeftDiffyPose());
        telemetry.addData("Right Axon", diffy.getRightDiffyPose());

        //Claw
        if (gamepad2.square) {
            claw.open();
        }
        if (gamepad2.triangle) {
            claw.close();
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();  // Horizontal offset
            double ty = result.getTy();  // Vertical offset

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);

            if (limelight.isTargetAligned(result)) {
                telemetry.addData("Alignment", "Target is centered!");
            } else {
                telemetry.addData("Alignment", "Target is NOT centered!");
            }
        } else {
            telemetry.addData("Limelight", "No valid data");
        }
        // Stop Limelight
        limelight.stopLimelight();

        telemetry.update();
    }
}
