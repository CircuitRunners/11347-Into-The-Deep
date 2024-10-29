package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight extends SubsystemBase {

    private Limelight3A limelight;
    private double alignmentTolerance;
    private Claw claw;

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

    public void aligningTarget(){//needs position of claw and orientaition of diffy
        LLResult result = getLatestResult();
        double tx = result.getTx();
        double ty = result.getTy();


        if(!isTargetAligned(result)){
            //if tx > currentTx: return code to move claw right
            //if tx < currentTx: return code to move claw left
            //if ty > currentTx: return code to move claw up
            //if ty < currentTx: return code to move claw dowm

        }
        //check whether the x coord range is greater than certain valuue:
        //if it is then rotate diffy
        //else: pass
        //run code to close claw

    }

    // This method doesnt work, do something similar though to figure out the orientation (based on the side lengths) @ZayneNair
//    public String getRectangleOrientation(LLResult result) {
//        if (result != null && result.isValid()) {
//            double tlong = result.getTlong();  // Longest side of the bounding box
//            double tshort = result.getTshort(); // Shortest side of the bounding box
//
//            if (tlong > tshort) {
//                return "Horizontal";
//            } else {
//                return "Vertical";
//            }
//        }
//        return "No valid data";
//    }


    //@ZanyeNair u need to add a function to determine if the block is in the correct orientation,
    // and if its not then give commands to the diffy to rotate so its in the correct spot

    //Also, make sure the code checks if the top length is greater or less than the side length to
    //figure out if its rotated correctly
    //     __            ____________
    //    |  |          |            |
    //    |  |    VS    |____________|
    //    |__|
}
