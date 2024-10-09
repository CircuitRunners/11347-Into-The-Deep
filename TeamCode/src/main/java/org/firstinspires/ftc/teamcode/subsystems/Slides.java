package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides extends SubsystemBase{
    DcMotorEx leftSlideMotor;
    DcMotorEx rightSlideMotor;

    public Slides(HardwareMap hardwareMap){
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

    }

    //Sets power to slides
    public void slidePower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    //sets power to 0
     public void slideBrakes(){
        slidePower(0);
    }
}
