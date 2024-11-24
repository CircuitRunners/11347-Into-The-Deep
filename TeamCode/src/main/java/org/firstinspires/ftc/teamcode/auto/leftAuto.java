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

import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected;
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrectedTwoPointOh;
import org.firstinspires.ftc.teamcode.subsystems.CRDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.support.Actions;
import org.firstinspires.ftc.teamcode.support.SleepCommand;



@Autonomous
public class leftAuto extends OpMode{

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Claw claw;
    private ArmCorrected arm;
    private Slides lift;
    private Diffy diffy;

    // Define key poses
    private Pose startPosition = new Pose(10.5, 81, Math.toRadians(0));
    private Pose preloadPos = new Pose(34, 81, Math.toRadians(0));
    private Pose sample1Pos = new Pose(25, 121, Math.toRadians(0));
    //private Point sample1CP = new Point(15, 105);
    private Pose placePos = new Pose(20, 122, Math.toRadians(135));
    private Pose sample2Pos = new Pose(25, 131.5, Math.toRadians(0));
    private Pose sample3Pos = new Pose(45.5, 125, Math.toRadians(90));
    //private Point sample3CP = new Point(46, 117);
    private Pose parkPos = new Pose(63, 98, Math.toRadians(270));
    private Point parkCP = new Point(62, 129);




    private PathChain preload, sample1Grab, sample1Place, sample2Grab, sample2Place, sample3Grab, sample3Place, park;
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(preloadPos)))
                .setConstantHeadingInterpolation(preloadPos.getHeading())
                .build();
        sample1Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPos), new Point(sample1Pos)))
                .setConstantHeadingInterpolation(sample1Pos.getHeading())
                .build();
        sample1Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos), new Point(placePos)))
                .setLinearHeadingInterpolation(sample1Pos.getHeading(), placePos.getHeading())
                .build();
        sample2Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(placePos), new Point(sample2Pos)))
                .setLinearHeadingInterpolation(placePos.getHeading(), sample2Pos.getHeading())
                .build();
        sample2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos), new Point(placePos)))
                .setLinearHeadingInterpolation(sample2Pos.getHeading(), placePos.getHeading())
                .build();
        sample3Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(placePos), new Point(sample3Pos)))
                .setLinearHeadingInterpolation(placePos.getHeading(), sample3Pos.getHeading())
                .build();
        sample3Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3Pos), new Point(placePos)))
                .setLinearHeadingInterpolation(sample3Pos.getHeading(), placePos.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(placePos), parkCP, new Point(parkPos)))
                .setLinearHeadingInterpolation(placePos.getHeading(), parkPos.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //Need to place preload
                    Actions.runBlocking(arm.toTopBar);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(claw.openClaw);
                    Actions.runBlocking(arm.toRestPos);
                    follower.followPath(sample1Grab);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //grab sample
                    Actions.runBlocking(arm.toGrabPos);
                    Actions.runBlocking(claw.closeClaw);
                    follower.followPath(sample1Place);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    //place sample in bucket
                    Actions.runBlocking(arm.toBasketPos);
                    Actions.runBlocking(diffy.endDiffy);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(claw.openClaw);
                    follower.followPath(sample2Grab);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    //grab sample
                    Actions.runBlocking(arm.toGrabPos);
                    Actions.runBlocking(diffy.startDiffy);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(claw.closeClaw);
                    follower.followPath(sample2Place);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    //place sample in bucket
                    Actions.runBlocking(arm.toBasketPos);
                    Actions.runBlocking(diffy.endDiffy);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(claw.openClaw);
                    ///follower.followPath(sample3Grab);
                    setPathState(8);//Skipping sample 3 for now
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //grab sample
                    Actions.runBlocking(arm.toGrabPos);
                    Actions.runBlocking(claw.closeClaw);
                    follower.followPath(sample3Place);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    //place sample
                    Actions.runBlocking(arm.toBasketPos);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(claw.openClaw);
                    follower.followPath(park);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //place sample
                    Actions.runBlocking(diffy.startDiffy);
                    Actions.runBlocking(arm.toRestPos);
                    Actions.runBlocking(claw.closeClaw);
                    follower.followPath(park);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //touch bar
                    Actions.runBlocking(arm.toTopBar);
                    setPathState(9);
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
        claw = new Claw(hardwareMap);
        lift = new Slides(hardwareMap);
        arm = new ArmCorrected(hardwareMap);
        diffy = new Diffy(hardwareMap);
        buildPaths();

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }




}
