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


@Autonomous
public class testRightAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // These are estimates and probably not great
    private Pose startPosition = new Pose(10.5, 62.5, Math.toRadians(0));
    private Pose preloadPosition = new Pose(35, 62.5, Math.toRadians(0));
    private Pose sample1Position = new Pose(60, 25, Math.toRadians(90));
    private Point sample1ControlPoint1 = new Point(35.5, 36);
    private Point sample1ControlPoint2 = new Point(63, 43);
    private Pose sample1ClipPosition = new Pose(20, 25, Math.toRadians(90));
    private Pose sample2Position = new Pose(60, 15, Math.toRadians(90));
    private Point sample2ControlPoint = new Point(62, 33);
    private Pose sample2ClipPosition = new Pose(20, 15, Math.toRadians(90));
    private Pose sample3Position = new Pose(45.5, 13, Math.toRadians(90));
    private Pose sample3ClipPosition = new Pose(20, 13, Math.toRadians(0));
    private Pose specimenGrabPosition = new Pose(14, 37, Math.toRadians(45));
    private Point specimen1ControlPoint = new Point(35, 41);
    private Pose specimen1PlacePosition = new Pose(34, 65, Math.toRadians(0));
    private Pose specimen2PlacePosition = new Pose(34, 63, Math.toRadians(0));
    private Pose specimen3PlacePosition = new Pose(34, 61, Math.toRadians(0));
    private Pose parkPosition = new Pose(10, 10, Math.toRadians(0));

    private PathChain preload, sample1, sample2, sample3Grab, sample3Place, specimen1Grab, specimen1Place, specimen2Grab, specimen2Place, specimen3Grab, specimen3Place, park;
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(preloadPosition)))
                .setConstantHeadingInterpolation(preloadPosition.getHeading())
                .build();
        sample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPosition), new Point(sample1Position), sample1ControlPoint1, sample1ControlPoint2))
                .setConstantHeadingInterpolation(sample1Position.getHeading())
                .addPath(new BezierLine(new Point(sample1Position), new Point(sample1ClipPosition)))
                .setConstantHeadingInterpolation(sample1ClipPosition.getHeading())
                .build();
        sample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample1ClipPosition), new Point(sample2Position), sample2ControlPoint))
                .setConstantHeadingInterpolation(sample2Position.getHeading())
                .addPath(new BezierLine(new Point(sample2Position), new Point(sample2ClipPosition)))
                .setConstantHeadingInterpolation(sample2ClipPosition.getHeading())
                .build();
        sample3Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2ClipPosition), new Point(sample3Position)))
                .setConstantHeadingInterpolation(sample3Position.getHeading())
                .build();
        sample3Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3Position), new Point(sample3ClipPosition)))
                .setConstantHeadingInterpolation(sample3ClipPosition.getHeading())
                .build();
        specimen1Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sample3ClipPosition), new Point(specimenGrabPosition), specimen1ControlPoint))
                .setConstantHeadingInterpolation(specimenGrabPosition.getHeading())
                .build();
        specimen1Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPosition), new Point(specimen1PlacePosition)))
                .setConstantHeadingInterpolation(specimen1PlacePosition.getHeading())
                .build();
        specimen2Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1PlacePosition), new Point(specimenGrabPosition)))
                .setConstantHeadingInterpolation(specimenGrabPosition.getHeading())
                .build();
        specimen2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPosition), new Point(specimen2PlacePosition)))
                .setConstantHeadingInterpolation(specimen2PlacePosition.getHeading())
                .build();
        specimen3Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2PlacePosition), new Point(specimenGrabPosition)))
                .setConstantHeadingInterpolation(specimenGrabPosition.getHeading())
                .build();
        specimen3Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenGrabPosition), new Point(specimen3PlacePosition)))
                .setConstantHeadingInterpolation(specimen3PlacePosition.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3PlacePosition), new Point(parkPosition)))
                .setConstantHeadingInterpolation(parkPosition.getHeading())
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
                    //do stuff to place preloaded
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //move arm back to rest
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
                    follower.followPath(sample3Place);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //place sample
                    follower.followPath(specimen1Grab);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    //grab specimen
                    follower.followPath(specimen1Place);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //place specimen
                    follower.followPath(specimen2Grab);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //grab specimen
                    follower.followPath(specimen2Place);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    //place Specimen
                    follower.followPath(specimen3Grab);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    //grab specimen
                    follower.followPath(specimen3Place);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    //place specimen
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
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }




}
