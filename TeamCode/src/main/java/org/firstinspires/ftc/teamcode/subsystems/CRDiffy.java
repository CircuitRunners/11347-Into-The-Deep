package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CRDiffy extends SubsystemBase {
    CRServo leftDiffyServo, rightDiffyServo;
    AnalogInput leftAxon, rightAxon;
    double startPosLeft, startPosRight;

    // PID constants for both left and right diffy
    private double kP = 0.1, kI = 0.01, kD = 0.005;
    private double leftIntegral = 0, leftPreviousError = 0;
    private double rightIntegral = 0, rightPreviousError = 0;

    // Constructor
    public CRDiffy(HardwareMap hardwareMap) {
        leftDiffyServo = hardwareMap.get(CRServo.class, "leftDiffyServo");
        rightDiffyServo = hardwareMap.get(CRServo.class, "rightDiffyServo");
        leftAxon = hardwareMap.get(AnalogInput.class, "leftAxon");
        rightAxon = hardwareMap.get(AnalogInput.class, "rightAxon");

        startPosLeft = leftAxon.getVoltage();
        startPosRight = rightAxon.getVoltage();
    }

    // PID control for left diffy
    public void setLeftDiffyPosition(double targetPosition) {
        double currentPos = getLeftDiffyPose();
        double error = targetPosition - currentPos;

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        leftIntegral += error;
        double iTerm = kI * leftIntegral;

        // Derivative term
        double derivative = error - leftPreviousError;
        double dTerm = kD * derivative;

        // PID output
        double output = pTerm + iTerm + dTerm;

        // Apply the PID output to the servo
        leftDiffyServo.setPower(-output); // Adjust sign if necessary
        leftPreviousError = error;
    }

    // PID control for right diffy
    public void setRightDiffyPosition(double targetPosition) {
        double currentPos = getRightDiffyPose();
        double error = targetPosition - currentPos;

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        rightIntegral += error;
        double iTerm = kI * rightIntegral;

        // Derivative term
        double derivative = error - rightPreviousError;
        double dTerm = kD * derivative;

        // PID output
        double output = pTerm + iTerm + dTerm;

        // Apply the PID output to the servo
        rightDiffyServo.setPower(output); // Adjust sign if necessary
        rightPreviousError = error;
    }

    // Simultaneous control of both diffy servos
    public void setDiffyPosition(double leftTargetPosition, double rightTargetPosition) {
        setLeftDiffyPosition(leftTargetPosition);
        setRightDiffyPosition(rightTargetPosition);
    }

    // Current left diffy position in degrees
    public double getLeftDiffyPose() {
        return leftAxon.getVoltage() / 3.3 * 360;
    }


    // Current right diffy position in degrees
    public double getRightDiffyPose() {
        return rightAxon.getVoltage() / 3.3 * 360;
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
    public void posDiffyFlip(){

        setLeftDiffyPosition(getLeftDiffyPose()/360+.5);
        setRightDiffyPosition(-getRightDiffyPose()/360+.5);
    }

    public void negDiffyFlip(){

        setLeftDiffyPosition(getLeftDiffyPose()/360-.5);
        setRightDiffyPosition(-getRightDiffyPose()/360-.5);
    }

    // Stop both diffy servos
    public void diffYBrake() {
        leftDiffyServo.setPower(0);
        rightDiffyServo.setPower(0);
    }
}
