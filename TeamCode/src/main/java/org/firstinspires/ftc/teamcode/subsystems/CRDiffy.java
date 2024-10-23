package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CRDiffy extends SubsystemBase{
    CRServo leftDiffyServo, rightDiffyServo;
    AnalogInput leftAxon, rightAxon;
    double startPosLeft, startPosRight;

    public CRDiffy(HardwareMap hardwareMap) {
            leftDiffyServo = hardwareMap.get(CRServo.class, "leftDiffyServo");
            rightDiffyServo = hardwareMap.get(CRServo.class, "rightDiffyServo");
            leftAxon = hardwareMap.get(AnalogInput.class, "leftAxon");
            rightAxon = hardwareMap.get(AnalogInput.class, "rightAxon");

            startPosLeft = leftAxon.getVoltage();
            startPosRight = rightAxon.getVoltage();
        }


    public void rotateDiffy (double power) {
        leftDiffyServo.setPower(-power);
        rightDiffyServo.setPower(power);

    }

    public void moveDiffy (double power) {
        leftDiffyServo.setPower(power);
        rightDiffyServo.setPower(power);
    }

    public void diffYBrake () {
        leftDiffyServo.setPower(0);
        rightDiffyServo.setPower(0);
    }

    public double getLeftDiffyPose() {
        return leftAxon.getVoltage() / 3.3 * 360 ;
    }

    public double getRightDiffyPose() {
        return rightAxon.getVoltage() / 3.3 * 360;
    }
}
