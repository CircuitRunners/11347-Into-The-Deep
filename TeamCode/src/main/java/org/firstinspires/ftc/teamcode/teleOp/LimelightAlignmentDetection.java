package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import com.qualcomm.hardware.limelightvision.LLResult;

@Disabled
@TeleOp(name = "Limelight Alignment Detection", group = "Sensor")
public class LimelightAlignmentDetection extends LinearOpMode {

    private Limelight limelight;
    private Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Limelight subsystem with a tolerance of 5 degrees
        limelight = new Limelight(hardwareMap, 5.0);
        claw = new Claw(hardwareMap);

        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Start the Limelight
        limelight.startLimelight();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();  // Horizontal offset
                double ty = result.getTy();  // Vertical offset

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);

                if (limelight.isTargetAligned(result)) {
                    telemetry.addData("Alignment", "Target is centered!");
                    claw.open();
                } else {
                    telemetry.addData("Alignment", "Target is NOT centered!");
                    claw.open();
                }
            } else {
                telemetry.addData("Limelight", "No valid data");
                claw.close();
                telemetry.addData("Claw", claw.getPosition());
            }

            telemetry.update();
        }

        // Stop Limelight
        limelight.stopLimelight();
    }
}
