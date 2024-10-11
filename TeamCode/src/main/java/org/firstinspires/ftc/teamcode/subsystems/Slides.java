package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides extends SubsystemBase{
    public enum SlidePosition {
        //set numbers for this
        DOWN(),
        AUTO(),
        SHORT(),
        MID(),
        HIGH();

        public int position;

        public SlidePositions (int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }
    DcMotorEx leftSlideMotor;
    DcMotorEx rightSlideMotor;

    public Slides(HardwareMap hardwareMap){
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

    }
    //even the motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //PID stuff
    leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    //reverse motor
    rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    //can change to break
    leftSlideMotor.setMode(DcMotor.ZeroPowerBehavior.float);
    rightSlideMotor.setMode(DcMotor.ZeroPowerBehavior.float);


    //Sets power to slides
    public void slidePower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    //sets power to 0
     public void slideBrakes(){
        slidePower(0);
    }

    public void resetPosition(){
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



}
