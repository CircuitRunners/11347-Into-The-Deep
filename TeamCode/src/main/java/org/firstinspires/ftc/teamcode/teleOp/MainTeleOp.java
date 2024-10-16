package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
@TeleOp
public class MainTeleOp extends CommandOpMode {
    //arm stuff
    public Arm arm;
    public Slides lift;
    public Diffy diffy;

    public Claw claw;


    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        arm = new Arm(hardwareMap);
        diffy = new Diffy(hardwareMap);
        lift = new Slides(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    @Override
    public void run() {
        super.run();
        arm.Power(gamepad2.left_stick_x); //justin i changed this to x
        //slide stuff

        lift.setLiftPower(gamepad2.left_stick_y);

        //diffy stuff

        diffy.moveDiffy(gamepad2.right_stick_y);
        diffy.rotateDiffy(gamepad2.right_trigger-gamepad2.left_trigger);

        //temp claw stuff
        if (gamepad2.square) {
            claw.open();
        }
        if (gamepad2.triangle) {
            claw.close();
        }

    }
}
