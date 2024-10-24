package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Disabled
@TeleOp
public class Test extends CommandOpMode {
public Claw claw;
    @Override
    public void initialize() {
         schedule(new BulkCacheCommand(hardwareMap));
         //arm = new Arm(hardwareMap);
         //ffy = new Diffy(hardwareMap);
         //lift = new Slides(hardwareMap);
         claw = new Claw(hardwareMap);
    }
    @Override
    public void run() {
        super.run();
        telemetry.addData("Servo Position", claw.getPosition());
        telemetry.update();

        //temp claw stuff
        if (gamepad2.square) {
            claw.open();
        }
        if (gamepad2.triangle) {
            claw.close();
        }
        claw.clawPosition(gamepad2.right_trigger);
    }
}
