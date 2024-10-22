package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp(name = "Limelight Alignment Detection", group = "Sensor")
public class LimelightAlignmentDetection extends LinearOpMode {

    private Limelight limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new Limelight(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            limelight.detectBlock();
        }
    }
}
