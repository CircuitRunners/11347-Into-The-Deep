package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SDiffy extends SubsystemBase {

    Servo leftDiffyServo, rightDiffyServo;
    double MAXPOS = 0.5;

    public SDiffy(HardwareMap hardwareMap) {
        leftDiffyServo = hardwareMap.get(Servo.class, "leftDiffyServo");
        rightDiffyServo = hardwareMap.get(Servo.class, "rightDiffyServo");
    }

    public void rotateDiffy (double power) {
        leftDiffyServo.setPosition(-(MAXPOS * power));
        rightDiffyServo.setPosition(MAXPOS * power);

    }

    public void moveDiffy (double power) {
        leftDiffyServo.setPosition(MAXPOS * power);
        rightDiffyServo.setPosition(MAXPOS * power);
    }
}
