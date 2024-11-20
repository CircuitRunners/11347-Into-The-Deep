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

import org.firstinspires.ftc.teamcode.subsystems.ArmCorrectedTwoPointOh;
import org.firstinspires.ftc.teamcode.subsystems.CRDiffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.support.Actions;
import org.firstinspires.ftc.teamcode.support.SleepCommand;


@Autonomous
public class rightAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Claw claw;
    private ArmCorrectedTwoPointOh arm;
    private Slides lift;
    private CRDiffy diffy;

    // These are estimates and probably not great
    private Pose startPosition = new Pose(10.5, 62.5, Math.toRadians(0));
    private Pose preloadPos = new Pose(35, 62.5, Math.toRadians(0));
    private Pose sample1GrabPos = new Pose(60, 25, Math.toRadians(90));
    private Point sample1GrabCP1 = new Point(35.5, 36);
    private Point sample1GrabCP2 = new Point(63, 43);
    private Pose sample1PlacePos = new Pose(20, 25, Math.toRadians(90));
    private Pose sample2GrabPos = new Pose(60, 15, Math.toRadians(90));
    private Point sample2GrabCP = new Point(62, 33);
    private Pose sample2PlacePos = new Pose(20, 15, Math.toRadians(90));
    private Pose sample3GrabPos = new Pose(45.5, 13, Math.toRadians(90));
    private Pose sample3PlacePos = new Pose(20, 13, Math.toRadians(0));
    private Pose specimenGrabPos = new Pose(14, 37, Math.toRadians(45));
    private Point specimen1GrabCP = new Point(35, 41);
    private Pose specimen1PlacePos = new Pose(34, 65, Math.toRadians(0));
    private Pose specimen2PlacePos = new Pose(34, 63, Math.toRadians(0));
    private Pose specimen3PlacePos = new Pose(34, 61, Math.toRadians(0));
    private Pose parkPos = new Pose(10, 10, Math.toRadians(0));

    private PathChain preload, sample1, sample2, sample3Grab, sample3Place, specimen1Grab, specimen1Place, specimen2Grab, specimen2Place, specimen3Grab, specimen3Place, park;
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(preloadPos)))
                .setConstantHeadingInterpolation(preloadPos.getHeading())
                .build();
        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPos), new Point(sample1GrabPos), sample1GrabCP1, sample1GrabCP2))
                .setConstantHeadingInterpolation(sample1GrabPos.getHeading())
                .addPath(new BezierLine(new Point(sample1GrabPos), new Point(sample1PlacePos)))
                .setConstantHeadingInterpolation(sample1PlacePos.getHeading())
                .build();
        sample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample1PlacePos), new Point(sample2GrabPos), sample2GrabCP))
                .setConstantHeadingInterpolation(sample2GrabPos.getHeading())
                .addPath(new BezierLine(new Point(sample2GrabPos), new Point(sample2PlacePos)))
                .setConstantHeadingInterpolation(sample2PlacePos.getHeading())
                .build();
        sample3Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2PlacePos), new Point(sample3GrabPos)))
                .setConstantHeadingInterpolation(sample3GrabPos.getHeading())
                .build();
        sample3Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3GrabPos), new Point(sample3PlacePos)))
                .setConstantHeadingInterpolation(sample3PlacePos.getHeading())
                .build();
        specimen1Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample3PlacePos), new Point(specimenGrabPos), specimen1GrabCP))
                .setConstantHeadingInterpolation(specimenGrabPos.getHeading())
                .build();
        specimen1Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPos), new Point(specimen1PlacePos)))
                .setConstantHeadingInterpolation(specimen1PlacePos.getHeading())
                .build();
        specimen2Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1PlacePos), new Point(specimenGrabPos)))
                .setConstantHeadingInterpolation(specimenGrabPos.getHeading())
                .build();
        specimen2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPos), new Point(specimen2PlacePos)))
                .setConstantHeadingInterpolation(specimen2PlacePos.getHeading())
                .build();
        specimen3Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2PlacePos), new Point(specimenGrabPos)))
                .setConstantHeadingInterpolation(specimenGrabPos.getHeading())
                .build();
        specimen3Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPos), new Point(specimen3PlacePos)))
                .setConstantHeadingInterpolation(specimen3PlacePos.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3PlacePos), new Point(parkPos)))
                .setConstantHeadingInterpolation(parkPos.getHeading())
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
                    //do stuff to place preloaded. This probably doesn't work
                    Actions.runBlocking(arm.toTopBar);
                    Actions.runBlocking(new SleepCommand(1));
                    //Actions.runBlocking(claw.openClaw);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //move arm back to rest
                    Actions.runBlocking(arm.toTopBar);
                    follower.followPath(sample1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(sample2);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(sample3Grab);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    //grab sample
                    //need to move diffy
                    //Actions.runBlocking(claw.closeClaw);
                    //maybe angle diffy up some
                    follower.followPath(sample3Place);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //place sample
                    //Actions.runBlocking(claw.openClaw);
                    follower.followPath(specimen1Grab);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    //grab specimen
                    //need to move diffy
                    //Actions.runBlocking(claw.closeClaw);
                    follower.followPath(specimen1Place);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //place specimen. this probably doesn't work
                    Actions.runBlocking(arm.toTopBar);
                    Actions.runBlocking(new SleepCommand(1));
                    //Actions.runBlocking(claw.openClaw);
                    Actions.runBlocking(arm.toTopBar);
                    follower.followPath(specimen2Grab);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //grab specimen
                    //Actions.runBlocking(claw.closeClaw);
                    follower.followPath(specimen2Place);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    //place Specimen. probably still doesn't work
                    Actions.runBlocking(arm.toTopBar);
                    Actions.runBlocking(new SleepCommand(1));
                    //Actions.runBlocking(claw.openClaw);
                    Actions.runBlocking(arm.toTopBar);
                    follower.followPath(specimen3Grab);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    //grab specimen
                    //Actions.runBlocking(claw.closeClaw);
                    follower.followPath(specimen3Place);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    //place specimen. probably doesn't work again
                    Actions.runBlocking(arm.toTopBar);
                    Actions.runBlocking(new SleepCommand(1));
                    //Actions.runBlocking(claw.openClaw);
                    Actions.runBlocking(arm.toTopBar);
                    follower.followPath(park);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
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
        arm = new ArmCorrectedTwoPointOh(hardwareMap);
        diffy = new CRDiffy(hardwareMap);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }




}
