package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Arm extends SubsystemBase {
    public enum ArmPositions{
        REST(5),
        MID(290),
        AUTO(320),
        BASKET_HIGH(415),
        HOVER_SUB(520),
        GRAB_SUB(575);

        public int position;

        ArmPositions(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    DcMotorEx armMotor;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private double VOLTAGE_WHEN_TUNED = 13.0;

    private double kP = 0.1,  kI = 0, kD = 0;

    private final static int UPPER_LIMIT = 600, MIDDLE_LIMIT = 290, LOWER_LIMIT = 35;

    private final static int ANGLE_1 = 000;//Robot side


    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        voltageComp = VOLTAGE_WHEN_TUNED / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
    }

    public void setPower(double power) {
        this.setPower(power, false);
    }
    public void setPower(double power, boolean transit) {
        if (atMiddleLimit()) {
            armMotor.setPower(power - 0.3);
        } else if (!atMiddleLimit() && !atLowerLimit()) {
            armMotor.setPower(power + 0.2); //320 mid pose Current angle - MIDDLE
        }
        if (!atMiddleLimit() && atLowerLimit()) {
            armMotor.setPower(power);
        }
    }

    public boolean atUpperLimit(){
        return getArmPosition() > UPPER_LIMIT;
    }
    public int getUpperLimit() {
        return UPPER_LIMIT;
    }
    public boolean atLowerLimit() {
        return getArmPosition() < LOWER_LIMIT;
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

    public void brake_power(){
        setPower(0);
    }

    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }

    public double getArmVelocity(){
        return armMotor.getVelocity();
    }
    public void resetLiftPosition(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVoltageComp(){
        return voltageComp;
    }
}
