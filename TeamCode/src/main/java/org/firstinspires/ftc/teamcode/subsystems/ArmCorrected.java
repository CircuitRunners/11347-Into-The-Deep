package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.support.RunAction;

@Config
public class ArmCorrected {
    public enum ArmPositions{
        REST(5),
        MID(290),
        AUTO(1400),
        BASKET_HIGH(1900),
        SPECIMEN(400),
        TOPBAR(1800),
        HOVER_SUB(534),
        GRAB_SUB(2000),
        GRAB_SAMPLE(3250);

        public int position;

        ArmPositions(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    private PIDController controller;
    public static double p = 0.006, i = 0, d = 0.0003;
    public static double f = 0.00;

    public static int target = 0;

    public DcMotorEx armMotor;

    public RunAction toTopBar, toGrabSample, toGrabPos, toRestPos, toBasketPos, toSpecimenPos, resetArmPosition, armAuto;

    public ArmCorrected(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        target = getCurrentPosition();

        toTopBar = new RunAction(this::toTopBar);
        toGrabPos = new RunAction(this::toGrabPos);
        toRestPos = new RunAction(this::toRestPos);
        toBasketPos = new RunAction(this::toBasketPos);
        toSpecimenPos = new RunAction(this::toSpecimenPos);
        resetArmPosition = new RunAction(this::resetArmPosition);
        armAuto = new RunAction(this::armAuto);
        toGrabSample = new RunAction(this::grabSample);
    }

    public void update() {
        controller.setPID(p, i, d);
        double armPos = getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target)) * f;

        double power = pid + ff;

        armMotor.setPower(power);
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public void setArmTarget(int new_target) {
        target = new_target;
    }

    public double getArmCurrent() {
        return armMotor.getCurrent(CurrentUnit.AMPS);
    }

    public int getArmTarget() {
        return target;
    }

    public void manual(int a) {
        setArmTarget(target + a * 5);
    }

    //    public static int TestingVar = 1000;
    public void toTopBar() {
        setArmTarget(ArmPositions.TOPBAR.position);
        //setArmTarget(TestingVar);
    }
    public void toGrabPos() {
        setArmTarget(ArmPositions.SPECIMEN.position);
    }
    public void toRestPos() {
        setArmTarget(ArmPositions.REST.position);
    }
    public void toBasketPos() {
        setArmTarget(ArmPositions.BASKET_HIGH.position);
    }
    public void toSpecimenPos() {
        setArmTarget(ArmPositions.SPECIMEN.position);
    }
    public void armAuto() {
        setArmTarget(ArmPositions.AUTO.position);
    }
    public void grabSample() {setArmTarget(ArmPositions.AUTO.position);}


    public void resetArmPosition() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
