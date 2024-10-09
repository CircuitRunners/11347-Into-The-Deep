package org.firstinspires.ftc.teamcode.teleOp;

public class MainTeleOp extends CommandOpMode {
    //arm stuff
    public Arm arm;
    arm.Power(gamepad2.left_stick_x); //justin i changed this to x

    //slide stuff
    public Slides slides;
    slides.slidePower(gamepad2.left_stick_y);

    //diffy stuff
    public Diffy diffy;
    diffy.moveDiffy(gamepad2.right_stick_y);
    diffy.rotateDiffy(gamepad2.right_trigger-gamepad2.left_trigger);

}
