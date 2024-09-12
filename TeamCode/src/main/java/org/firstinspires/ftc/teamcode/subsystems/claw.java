package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class claw extends SubsystemBase {
    Servo leftservo, rightservo;

    public claw(HardwareMap h) {
        leftservo = h.get(Servo.class, "leftservo");
        rightservo = h.get(Servo.class, "rightservo");

    }

    public void rotation(){
        leftservo.setPosition(0.25);
        rightservo.setPosition(-0.25);

    }

    public void spin(){
        leftservo.setPosition(0.25);
        rightservo.setPosition(0.25);

    }
}
