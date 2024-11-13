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
public class betterTestAuto extends OpMode {
//Auto to test tuning accuracy
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define key poses
    private Pose startPosition = new Pose(10, 111, Math.toRadians(0));
    private Pose pos1 = new Pose(111, 111, Math.toRadians(-90));
    private Pose pos2 = new Pose(111, 30, Math.toRadians(180));
    private Pose pos3 = new Pose(33, 33, Math.toRadians(90));
    private Pose pos4 = new Pose(33, 111, Math.toRadians(0));



    private PathChain line1, line2, line3, line4, line5, line6;
    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPosition), new Point(pos1)))
                .setConstantHeadingInterpolation(pos1.getHeading())
                .build();
        line2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pos1), new Point(pos2)))
                .setConstantHeadingInterpolation(pos2.getHeading())
                .build();
        line3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pos2), new Point(pos3)))
                .setConstantHeadingInterpolation(pos3.getHeading())
                .build();
        line4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pos3), new Point(pos4)))
                .setConstantHeadingInterpolation(pos4.getHeading())
                .build();
        line5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pos4), new Point(pos1)))
                .setConstantHeadingInterpolation(pos1.getHeading())
                .build();
        line6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pos4), new Point(startPosition)))
                .setConstantHeadingInterpolation(startPosition.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {//goes in a square around center 3 times then goes back to start
        switch (pathState) {
            case 0:
                follower.followPath(line1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
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
                    follower.followPath(line5);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(line4);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(line5);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(line4);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(line5);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(line6);
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
