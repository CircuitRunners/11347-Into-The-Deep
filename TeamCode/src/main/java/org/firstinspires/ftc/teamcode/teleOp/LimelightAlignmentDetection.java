package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "Limelight Alignment Detection", group = "Sensor")
public class LimelightAlignmentDetection extends LinearOpMode {

    private Limelight3A limelight;
    private Claw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        claw = new Claw(hardwareMap);

        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Start the Limelight
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        // Tolerance for center alignment (adjust if needed)
        double alignmentTolerance = 5.0;  // +/-2 degrees tolerance for tx and ty

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();  // Horizontal offset
                double ty = result.getTy();  // Vertical offset

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);

                // Check if both tx and ty are within the tolerance range for alignment
                if (Math.abs(tx) <= alignmentTolerance && Math.abs(ty) <= alignmentTolerance) {
                    telemetry.addData("Alignment", "Target is centered!");
                } else {
                    telemetry.addData("Alignment", "Target is NOT centered!");
                    claw.getPosition();
                }
            } else {
                telemetry.addData("Limelight", "No valid data");
                claw.getPosition();
            }

            telemetry.update();
        }

        // Stop Limelight
        limelight.stop();
    }
}
