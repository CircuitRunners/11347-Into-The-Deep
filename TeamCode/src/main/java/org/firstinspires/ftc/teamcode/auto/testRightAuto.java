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

    // Define key poses
    private Pose startPosition = new Pose(10.5, 62.5, Math.toRadians(0));
    private Pose preloadPosition = new Pose(35, 62.5, Math.toRadians(0));
    private Pose sample1Position = new Pose(60, 25, Math.toRadians(90));
    private Point sample1ControlPoint1 = new Point(35.5, 36);
    private Point sample1ControlPoint2 = new Point(63, 43);
    private Pose sample1ClipPosition = new Pose(20, 25, Math.toRadians(90));



    private PathChain preload, sample1;
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
