package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    Servo servo;

    public Claw(HardwareMap h) {
        servo = h.get(Servo.class, "Claw Servo");

    }

    public void open(){
        //Position is probably wrong
        servo.setPosition(.25);
    }

    public void close() {
        servo.setPosition(0);
    }

    public void clawPosition(double position) {
        servo.setPosition(position);
    }

}
