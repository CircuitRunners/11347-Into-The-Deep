package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.support.RunAction;

public class SlidesPID{
    //TODO Rename References
    public enum SlidePositions {
        STAGE_0(-30), // Ground
        AUTO(-200), // Low Bar
        STAGE_1(-1070), // Low Bucket
        STAGE_2(-2200), // High Bar
        STAGE_3(-3130); // High Bucket

        public int position;

        SlidePositions(int position){
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }
    private int UPPER_LIMIT = -3180, LOWER_LIMIT = -50;
    DcMotorEx leftSlideMotor;
    DcMotorEx rightSlideMotor;
    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.0001;
    public static double f = 0.00;

    public static double target = 0;
    private VoltageSensor voltageSensor;
    private double voltageComp;
    private double VOLTAGE_WHEN_LIFT_TUNED = 13.0;
    public RunAction scoring, rest;
    public SlidesPID(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        //target = getLiftPosition();
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

        //even the motors
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //PID stuff
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse motor
        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Negate the gravity when stopped ?
        //TODO gravity PID coefficients?
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_LIFT_TUNED / voltageSensor.getVoltage();
        scoring = new RunAction(this::scoring);
        rest = new RunAction(this::rest);
    }

    public void update() {
        controller.setPID(p, i, d);
        double armPos = getLiftPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target)) * f;

        double power = pid + ff;

        rightSlideMotor.setPower(power);
        leftSlideMotor.setPower(power);
    }

    public void setLiftPower(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
    public void setLiftTarget(int new_target) {
        target = new_target;
    }
    public void brake_power(){
        setLiftPower(0);
    }

    public double getLiftPosition(){
        return rightSlideMotor.getCurrentPosition();
    }

    public double getLiftVelocity(){
        return rightSlideMotor.getVelocity();
    }

    public boolean atUpperLimit(){
        return getLiftPosition() < UPPER_LIMIT;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() > LOWER_LIMIT;
    }

    public void setLeftPower(double power) {
        leftSlideMotor.setPower(power);
    }

    public void setRightMotor(double power) {
        rightSlideMotor.setPower(power);
    }
    public void scoring() {setLiftTarget(SlidePositions.STAGE_2.getPosition());}
    public void rest() {setLiftTarget(SlidePositions.STAGE_0.getPosition());}

    public void resetLiftPosition() {
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVoltageComp() {
        return voltageComp;
    }
}
