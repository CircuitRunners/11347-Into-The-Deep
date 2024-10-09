package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Diffy extends SubsystemBase{
    CRServo leftDiffyServo, rightDiffyServo;
    public Diffy (HardwareMap hardwareMap) {
            leftDiffyServo = hardwareMap.get(Servo.class, "leftDiffyServo");
            rightDiffyServo = hardwareMap.get(Servo.class, "rightDiffyServo");
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

    }?
}
