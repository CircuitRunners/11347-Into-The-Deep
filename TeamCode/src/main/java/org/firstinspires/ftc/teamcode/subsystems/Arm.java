package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Arm extends SubsystemBase{
    DcMotorEx = armMotor;
    pub
    public Arm(HardwareMap hardwareMap){
        this(hardwareMap);
        armMotor = hardwareMap.get(ServoImplEx.class, "armmotor");
    }

    //TeleOp
    public void Power(double power) {
        armMotor.setPower(power);
    }



    //autonomous stuff
    public void upPower() {
        armMotor.setPower(0.5);
    }
    public void downPower() {
        armMotor.setPower(-0.5);
    }


    mainMotor.setPower(0.5); 
}
