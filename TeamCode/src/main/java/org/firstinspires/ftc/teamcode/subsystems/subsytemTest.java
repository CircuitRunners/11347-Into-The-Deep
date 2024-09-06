package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subsytemTest extends SubsystemBase {
    private DcMotorEx motor;

    public subsytemTest(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

    }

    public void spin() {
        motor.setPower(1);
    }

    public void stop() {
        motor.setPower(0);
    }
}
