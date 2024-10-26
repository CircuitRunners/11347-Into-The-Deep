package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Arm extends SubsystemBase {
    public enum ArmPositions{
        REST(0),
        HOVER_SUB(0),
        GRAB_SUB(0),
        BASKET_LOW(0),
        BASKET_HIGH(0),
        LOWER_BAR(0),
        UPPER_BAR(0);

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

    private final static int UPPER_LIMIT = 670, MIDDLE_LIMIT = 290, LOWER_LIMIT = 35;

    private final static int ANGLE_1 = 000;//Robot side


    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        voltageComp = VOLTAGE_WHEN_TUNED / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
    }

    public void setPower(double power) {
        if (atMiddleLimit()) {
            armMotor.setPower(power - 0.3);
        } else if (!atMiddleLimit()) {
            armMotor.setPower(power + 0.2); //320 mid pose Current angle - MIDDLE
        }
    }

    public boolean atUpperLimit(){
        return getArmPosition() > UPPER_LIMIT;
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

    public double getLiftVelocity(){
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
