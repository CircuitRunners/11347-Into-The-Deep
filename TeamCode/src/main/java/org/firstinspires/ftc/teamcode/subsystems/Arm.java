package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Arm extends SubsystemBase {
    DcMotorEx armMotor;

    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
    }

    //TeleOp
    public void Power(double power) {
        armMotor.setPower(power);
    }

    //autonomous stuff
    // public void upPower() {
    //     armMotor.setPower(0.5);
    // }
    // public void downPower() {
    //     armMotor.setPower(-0.5);
    // }

    public void upPower() {
        armMotor.setPower(0.5);
    }
    public void downPower() {
        armMotor.setPower(-0.5);
    }
}
