package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CRDiffy extends SubsystemBase {
    CRServo leftDiffyServo, rightDiffyServo;
    AnalogInput leftAxonEncoder, rightAxonEncoder;
    double startPosLeft, startPosRight;

    // Constructor
    public CRDiffy(HardwareMap hardwareMap) {
        leftDiffyServo = hardwareMap.get(CRServo.class, "leftDiffyServo");
        rightDiffyServo = hardwareMap.get(CRServo.class, "rightDiffyServo");

        leftAxonEncoder = hardwareMap.get(AnalogInput.class, "leftAxon");
        rightAxonEncoder = hardwareMap.get(AnalogInput.class, "rightAxon");

        startPosLeft = leftAxonEncoder.getVoltage();
        startPosRight = rightAxonEncoder.getVoltage();
    }

    public void setPosition(double leftTarget, double rightTarget) {
        //take pos of servos, have it move to the target
        //Check the current pose of the servo vs the target, and use that to update the power of the servo
    }

    // Current left diffy position in degrees
    public double getLeftDiffyPose() {
        return leftAxonEncoder.getVoltage() / 3.3 * 360;
    }

    // Current right diffy position in degrees
    public double getRightDiffyPose() {
        return rightAxonEncoder.getVoltage() / 3.3 * 360;
    }

    // Manual rotation control
    public void rotateDiffy(double power) {
        leftDiffyServo.setPower(-power);
        rightDiffyServo.setPower(power);
    }

    // Move both diffys together
    public void moveDiffy(double power) {
        leftDiffyServo.setPower(power);
        rightDiffyServo.setPower(power);
    }

    // Stop both diffy servos
    public void diffYBrake() {
        leftDiffyServo.setPower(0);
        rightDiffyServo.setPower(0);
    }
}
