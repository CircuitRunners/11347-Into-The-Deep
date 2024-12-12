//package org.firstinspires.ftc.teamcode.auto;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.button.Trigger;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.commands.presets.ArmToScoringCommand;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
//
//@Autonomous(group="1")
//public class timebasedParkRight extends CommandOpMode {
//
//    private Drivebase db;
//    private Arm arm;
//    private Claw claw;
//    private ElapsedTime timer;
//    boolean runArm = false;
//    private boolean isFirst = true;
//
//    @Override
//    public void initialize() {
//        db = new Drivebase(hardwareMap);
//        arm = new Arm(hardwareMap);
//        claw = new Claw(hardwareMap);
//        timer = new ElapsedTime();
//
//        new Trigger(() -> runArm)
//                .whenActive(new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.AUTO)
//                        .withTimeout(2000));
//
//
//        telemetry.addLine("Init Done");
//        telemetry.update();
//    }
//
//    @Override
//    public void run() {
//        super.run();
//
//        if (isFirst) {
//            timer.reset();
//            isFirst = false;
//        }
//
//        // Set the robot to drive forward for 2 seconds
//        if (timer.seconds() < 1.5) { //1.8
//            db.drive(0, 0.5, 0); // Drive forward at half speed
//        } else {
//            db.drive(0, 0, 0);
//        }
//        // Display the timer on the telemetry
//        telemetry.addData("Arm >", arm.getArmPosition());
//        telemetry.addData("Timer >", timer.seconds());
//        telemetry.update();
//    }
//}
