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
public class leftAuto extends OpMode{

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define key poses
    private Pose startPosition = new Pose(10.5, 62.5, Math.toRadians(0));
    private Pose preloadPos = new Pose(35, 78, Math.toRadians(0));
    private Pose sample1Pos = new Pose(34, 121, Math.toRadians(-180));
    private Point sample1CP = new Point(15, 105);
    private Pose placePos = new Pose(18.5, 124, Math.toRadians(-225));
    private Pose sample2Pos = new Pose(34, 131.5, Math.toRadians(-180));
    private Pose sample3Pos = new Pose(45.5, 132, Math.toRadians(-90));
    private Point sample3CP = new Point(46, 117);
    private Pose parkPos = new Pose(60, 96, Math.toRadians(-90));





    private PathChain preload, sample1Grab, sample1Place, sample2Grab, sample2Place, sample3Grab, sample3Place, park;
    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(preloadPos)))
                .setConstantHeadingInterpolation(preloadPos.getHeading())
                .build();
        sample1Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPos), new Point(sample1Pos), sample1CP))
                .setConstantHeadingInterpolation(sample1Pos.getHeading())
                .build();
        sample1Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1Pos), new Point(placePos)))
                .setConstantHeadingInterpolation(placePos.getHeading())
                .build();
        sample2Grab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(placePos), new Point(sample2Pos)))
                .setConstantHeadingInterpolation(sample2Pos.getHeading())
                .build();
        sample2Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2Pos), new Point(placePos)))
                .setConstantHeadingInterpolation(placePos.getHeading())
                .build();
        sample3Grab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(placePos), new Point(sample3Pos), sample3CP))
                .setConstantHeadingInterpolation(sample3Pos.getHeading())
                .build();
        sample3Place = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3Pos), new Point(placePos)))
                .setConstantHeadingInterpolation(placePos.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(placePos), new Point(parkPos)))
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
                    //Need to place preload
                    follower.followPath(sample1Grab);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    //grab sample
                    follower.followPath(sample1Place);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    //place sample in bucket
                    follower.followPath(sample2Grab);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    //grab sample
                    follower.followPath(sample2Place);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    //place sample in bucket
                    follower.followPath(sample3Grab);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //grab sample
                    follower.followPath(sample3Place);
                    setPathState(6);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    //place sample
                    follower.followPath(park);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //touch bar
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
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }




}
