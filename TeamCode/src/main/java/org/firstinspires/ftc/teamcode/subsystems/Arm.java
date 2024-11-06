package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.support.RunAction;

public class Arm extends SubsystemBase {
    public enum ArmPositions{
        REST(5),
        MID(290),
        AUTO(410),
        BASKET_HIGH(400),
        HOVER_SUB(520),
        GRAB_SUB(590);

        public int position;

        ArmPositions(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    private PIDController controller;

    private static double kP = 0.022;
    private static double kI = 0;
    private static double kD = 0.0015;
    private static double f = 0.15;
    private static double powerMultiplier = 0.5;

    private final double ticks_in_degree = 700 / 100.0;

    DcMotorEx armMotor;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private double VOLTAGE_WHEN_TUNED = 13.0;

    private final static int UPPER_LIMIT = 600, MIDDLE_LIMIT = 290, LOWER_LIMIT_ONE = 45, LOWER_LIMIT_TWO = 35;

    private final static int        UPPER_ONE = 390,            UPPER_TWO = 470,            UPPER_THREE = 540,          UPPER_FOUR = 610;           // OUTSIDE ROBOT
    private final static double     UPPER_ONE_POWER = -0.24,    UPPER_TWO_POWER = -0.34,    UPPER_THREE_POWER = -0.45,  UPPER_FOUR_POWER = -0.50;   // OUTSIDE ROBOT
    private final static int        LOWER_ONE = 50,             LOWER_TWO = 250,            LOWER_THREE = 290;          // INSIDE ROBOT
    private final static double     LOWER_ONE_POWER = 0.45,     LOWER_TWO_POWER = 0.35,     LOWER_THREE_POWER = 0.14;   // INSIDE ROBOT

    public RunAction armRESTPos, armMIDPos, armAUTOPos, armBASKET_HIGHPos, armHOVER_SUBPos, armGRAB_SUBPos;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = new PIDController(kP, kI, kD);

        armRESTPos = new RunAction(this::armRESTPos);
        armMIDPos = new RunAction(this::armMIDPos);
        armAUTOPos = new RunAction(this::armAUTOPos);
        armBASKET_HIGHPos = new RunAction(this::armBASKET_HIGHPos);
        armHOVER_SUBPos = new RunAction(this::armHOVER_SUBPos);
        armGRAB_SUBPos = new RunAction(this::armGRAB_SUBPos);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_TUNED / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
    }

    public void setPowerTesting(double power) { //setPower
        if (atMiddleLimit() && !atUpperOne()) {
            armMotor.setPower(power - 0.3); //0.3
        } else if (atMiddleLimit() && atUpperOne()) {
            armMotor.setPower(power - 0.3);
        } else if (!atMiddleLimit() && !atLowerLimit()) {
            armMotor.setPower(power + 0.2); //320 mid pose Current angle - MIDDLE
        }
        if (!atMiddleLimit() && atLowerLimit()) {
            armMotor.setPower(power);
        }
    }

    public void armPIDTest(int target) {
        controller.setPID(kP, kI, kD);
        int armPos = getArmPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        armMotor.setPower(power * powerMultiplier);
    }

    public void armRESTPos() {
        armPIDTest(ArmPositions.REST.position);
    }

    public void armMIDPos() {
        armPIDTest(ArmPositions.MID.position);
    }

    public void armAUTOPos() {
        armPIDTest(ArmPositions.AUTO.position);
    }

    public void armBASKET_HIGHPos() {
        armPIDTest(ArmPositions.AUTO.position);
    }

    public void armHOVER_SUBPos() {
        armPIDTest(ArmPositions.AUTO.position);
    }

    public void armGRAB_SUBPos() {
        armPIDTest(ArmPositions.AUTO.position);
    }

    public void setPower(double power) { //setPowerTesting
        armMotor.setPower(power + estimateArmPos());
    }

    public void setPowerActual(double power) {
        armMotor.setPower(power);
    }

    public boolean atUpperLimit(){
        return getArmPosition() > UPPER_LIMIT;
    }

    public boolean atLowerLimit() {
        return getArmPosition() < LOWER_LIMIT_ONE;
    }

    public boolean atUpperOne() {
        return getArmPosition() < UPPER_ONE;
    }

    public boolean atLowerOne() {
        return getArmPosition() > LOWER_ONE;
    }

    public boolean atMiddleLimit() {
        boolean state = false;
        if (getArmPosition() > MIDDLE_LIMIT) {
            state = true;
        } else if (getArmPosition() < MIDDLE_LIMIT) {
            state = false;
        }

        return state;
    }

    public double estimateArmPos() {
        double armPosition = getArmPosition();

        // No power if arm position is below LOWER_LIMIT
        if (armPosition < LOWER_LIMIT_ONE) {
            if (armPosition < LOWER_LIMIT_TWO) {
                return 0;
            } else {
                return 0.15;
            }
        }

        // Inside robot interpolation (positive power)
        if (armPosition <= LOWER_ONE) {
            return LOWER_ONE_POWER;
        } else if (armPosition <= LOWER_TWO) {
            return interpolate(armPosition, LOWER_ONE, LOWER_TWO, LOWER_ONE_POWER, LOWER_TWO_POWER);
        } else if (armPosition <= LOWER_THREE) {
            return interpolate(armPosition, LOWER_TWO, LOWER_THREE, LOWER_TWO_POWER, LOWER_THREE_POWER);
        }

        // Outside robot interpolation (negative power)
        if (armPosition <= UPPER_ONE) {
            return LOWER_THREE_POWER;
        } else if (armPosition <= UPPER_TWO) {
            return interpolate(armPosition, UPPER_ONE, UPPER_TWO, UPPER_ONE_POWER, UPPER_TWO_POWER);
        } else if (armPosition <= UPPER_THREE) {
            return interpolate(armPosition, UPPER_TWO, UPPER_THREE, UPPER_TWO_POWER, UPPER_THREE_POWER);
        } else if (armPosition <= UPPER_FOUR) {
            return interpolate(armPosition, UPPER_THREE, UPPER_FOUR, UPPER_THREE_POWER, UPPER_FOUR_POWER);
        } else {
            return UPPER_FOUR_POWER;
        }
    }

    // Helper method for linear interpolation
    private double interpolate(double position, int start, int end, double startPower, double endPower) {
        double ratio = (position - start) / (double) (end - start);
        return startPower + ratio * (endPower - startPower);
    }

    public void brake_power(){
        setPower(0);
    }
    public void brake_power(boolean state) {
        if (state) {
            setPower(0.2);
        } else {
            setPower(-0.2);
        }
    }

    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }

    public double getArmVelocity(){
        return armMotor.getVelocity();
    }
    public void resetArmPosition(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVoltageComp(){
        return voltageComp;
    }
}
