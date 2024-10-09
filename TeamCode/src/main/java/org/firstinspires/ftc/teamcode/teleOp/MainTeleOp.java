package org.firstinspires.ftc.teamcode.teleOp;

public class MainTeleOp extends CommandOpMode {
    //arm stuff
    public Arm arm;
    arm.Power(gamepad2.left_stick_x); //justin i changed this to x

    //slide stuff
    public Slides slides;
    slides.slidePower(gamepad2.left_stick_y);




}
