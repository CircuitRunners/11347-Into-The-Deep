package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.support.Actions;
import org.firstinspires.ftc.teamcode.support.SleepCommand;


@Autonomous
public class testAuto extends OpMode{//Auto just to test pathing and mechanisms to see if im doing it right

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Claw claw;
    private Arm arm;
    private Slides lift;

    // Define key poses
    private Pose startPosition = new Pose(10.5, 63, Math.toRadians(0));
    private Pose pos1 = new Pose(35, 63, Math.toRadians(0));
    private Pose pos2 = new Pose(36, 23, Math.toRadians(0));
    private Point pos2CP = new Point(10, 30);
    private Pose pos3 = new Pose(14, 37, Math.toRadians(45));
    private Point pos3CP = new Point(40, 47);
    private Pose pos4 = new Pose(35, 63, Math.toRadians(0));
    private Pose pos5 = new Pose(60, 96, Math.toRadians(-90));
    private Point pos5CP = new Point(14, 120);



    private PathChain line1, line2, line3, line4;
    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(pos1)))
                .setConstantHeadingInterpolation(pos1.getHeading())
                .build();
        line2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pos1), new Point(pos2), pos2CP))
                .setConstantHeadingInterpolation(pos2.getHeading())
                .addPath(new BezierCurve(new Point(pos2), new Point(pos3), pos3CP))
                .setConstantHeadingInterpolation(pos3.getHeading())
                .build();
        line3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pos3), new Point(pos4)))
                .setConstantHeadingInterpolation(pos4.getHeading())
                .build();
        line4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pos4), new Point(pos5), pos5CP))
                .setConstantHeadingInterpolation(pos5.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armMIDPos);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(arm.armRESTPos);
                    follower.followPath(line2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    Actions.runBlocking(claw.openClaw);
                    Actions.runBlocking(claw.closeClaw);
                    follower.followPath(line3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line4);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    //slide test stuff
                    Actions.runBlocking(arm.armAUTOPos);
                    setPathState(5);
                }
                break;
            default:
                requestOpModeStop();
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPosition);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }




}
