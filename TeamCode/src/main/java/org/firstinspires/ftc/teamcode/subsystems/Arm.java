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

    private double kP = 0.1,  kI = 0.01, kD = 0.005;

    private double leftIntegral = 0, leftPreviousError = 0;
    private double rightIntegral = 0, rightPreviousError = 0;


    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_TUNED / voltageSensor.getVoltage();
    }

    @Override
    public void periodic(){
        // happens every loop
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public void setPosition(double targetPosition){
        double currentPos = getArmPosition();
        double error = targetPosition - currentPos;

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        rightIntegral += error;
        double iTerm = kI * rightIntegral;

        // Derivative term
        double derivative = error - rightPreviousError;
        double dTerm = kD * derivative;

        // PID output
        double output = pTerm + iTerm + dTerm;

        // Apply the PID output to the servo
        setPower(output); // Adjust sign if necessary
        rightPreviousError = error;
    }

    public void brake_power(){
        setPower(0);
    }

    public double getArmPosition(){
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
