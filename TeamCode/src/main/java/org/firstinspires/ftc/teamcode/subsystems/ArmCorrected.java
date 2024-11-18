package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.RunAction;

@Config
public class ArmCorrected {
    public enum ArmPositions{
        REST(5),
        MID(290),
        AUTO(410),
        BASKET_HIGH(200),
        SPECIMEN(500),
        TOPBAR(328),
        HOVER_SUB(534),
        GRAB_SUB(605);

        public int position;

        ArmPositions(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    public RunAction toTopBar, toGrabPos;
    private Telemetry telemetry;

    private DcMotorEx armMotor;
    public PIDController ArmPID;

    public static int target;
    public static double p = 0.015, i = 0, d = 0.0008;
    public static double f = 0.07;

    public ArmCorrected(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmPID = new PIDController(p, i, d);

        toTopBar = new RunAction(this::toTopBar);
        toGrabPos = new RunAction(this::toGrabPos);
    }

    public void toTopBar() {
        setTarget(ArmPositions.TOPBAR.getPosition());
    }

    public void toGrabPos() {
        setTarget(ArmPositions.SPECIMEN.getPosition());
    }

    public void updatePIDF() {
        ArmPID.setPID(p, i, d);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double pid = ArmPID.calculate(getArmPosition(), target);
        double ticks_in_degrees = 0.0;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;

        armMotor.setPower(power);

        telemetry.addData("Arm pos", getArmPosition());
        telemetry.addData("Arm target", target);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public int getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public void init() {
        ArmPID.setPID(p, i, d);
    }

    public void start() {
        target = 0;
    }
}
