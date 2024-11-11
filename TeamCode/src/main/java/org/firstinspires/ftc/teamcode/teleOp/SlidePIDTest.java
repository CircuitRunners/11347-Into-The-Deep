package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.support.PID;

@Disabled
@TeleOp
@Config
public class SlidePIDTest extends LinearOpMode {

    DcMotor leftSlide;
    DcMotor rightSlide;
    private Slides lift;

    // PID controller for the lift system
    public static PID pid = new PID(
            0, 0, 0);

    @Override
    public void runOpMode() {

        leftSlide = hardwareMap.dcMotor.get("leftSlideMotor");
        rightSlide = hardwareMap.dcMotor.get("rightSlideMotor");
        lift = new Slides(hardwareMap);

        // Wait for start button press
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get the current position of the right slide
            double currentPosition = lift.getLiftPosition();

            // Display the current position on telemetry
            telemetry.addData("Slide Position: ", currentPosition);
            telemetry.addData("P", pid.kP);
            telemetry.addData("I", pid.kI);
            telemetry.addData("D", pid.kD);
            telemetry.addData("kP Control: ", "❌ and ⭕");
            telemetry.addData("kI Control: ", "⬛ and 🔼");
            telemetry.addData("kD Control: ", "Dpad ⬅️ and ➡️");
            telemetry.update();

            // Adjust PID coefficients using gamepad inputs
            if (gamepad2.cross) pid.kP += 0.01;
            if (gamepad2.circle) pid.kP -= 0.01;
            if (gamepad2.square) pid.kI += 0.01;
            if (gamepad2.triangle) pid.kI -= 0.01;
            if (gamepad2.dpad_left) pid.kD += 0.01;
            if (gamepad2.dpad_right) pid.kD -= 0.01;

            // Move the lift up/down based on D-Pad inputs, with PID controlling the motion
            if (gamepad2.dpad_down) {
                double targetPosition = currentPosition - 10; // Move down by 10 encoder ticks
                double power = pid.runPID(currentPosition, targetPosition);
                lift.setLiftPower(power);
            } else if (gamepad2.dpad_up) {
                double targetPosition = currentPosition + 10; // Move up by 10 encoder ticks
                double power = pid.runPID(currentPosition, targetPosition);
                lift.setLiftPower(power);
            } else {
                // Manual control with left stick if no PID movement is occurring
                lift.setLiftPower(gamepad2.left_stick_y);
            }
        }
    }
}
