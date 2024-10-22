package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight extends SubsystemBase {

    private Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void detectBlock() {
        // Set telemetry update interval
        telemetry.setMsTransmissionInterval(11);

        // Start the Limelight
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        // Tolerance for center alignment (adjust if needed)
        double alignmentTolerance = 5.0;  // +/- 5 degrees tolerance for tx and ty

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
            }
        } else {
            telemetry.addData("Limelight", "No valid data");
        }

        telemetry.update();
        limelight.stop();
    }
}
