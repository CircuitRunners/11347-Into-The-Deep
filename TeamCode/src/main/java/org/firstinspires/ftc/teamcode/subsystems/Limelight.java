package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight extends SubsystemBase {

    private Limelight3A limelight;
    private double alignmentTolerance;

    public Limelight(HardwareMap hardwareMap, double tolerance) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        alignmentTolerance = tolerance;
    }

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public boolean isTargetAligned(LLResult result) {
        if (result != null && result.isValid()) {
            double tx = result.getTx();  // Horizontal offset
            double ty = result.getTy();  // Vertical offset
            return Math.abs(tx) <= alignmentTolerance && Math.abs(ty) <= alignmentTolerance;
        }
        return false;
    }
}
