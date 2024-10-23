package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Slides extends SubsystemBase{
    public enum SlidePositions {
        DOWN(0),
        AUTO(100),
        SHORT(200),
        MID(300),
        HIGH(400);

        public int position;

        SlidePositions(int position){
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }
    private int UPPER_LIMIT = 2950;
    DcMotorEx leftSlideMotor;
    DcMotorEx rightSlideMotor;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private double VOLTAGE_WHEN_LIFT_TUNED = 13.0;

    public Slides(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

        //even the motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //PID stuff
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse motor
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Negate the gravity when stopped
        //TODO gravity PID coefficients?
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // change to brake if bad
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_LIFT_TUNED / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
    }

    public void setLiftPower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    public void brake_power(){
        setLiftPower(0);
    }

    public double getLiftPosition(){
        return rightSlideMotor.getCurrentPosition();
    }

    public double getLiftVelocity(){
        return leftSlideMotor.getVelocity();
    }

    public boolean atUpperLimit(){
        return getLiftPosition() > UPPER_LIMIT;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() < 5;
    }

    public void setLeftPower (double power) {
        leftSlideMotor.setPower(power);
    }

    public void setRightMotor (double power) {
        rightSlideMotor.setPower(power);
    }

    public void resetLiftPosition(){
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVoltageComp(){
        return voltageComp;
    }
}
