package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double toLineShort = 6;
    public static double toLineLong = 2.5;
    public static double toMiddle = 28.75;
    public static double parkRight = 40;
    public static double parkLeft = 97;
    public static double prepPark = 20;//this is for far left and far right
    public static double prepParkMid = 4;//this is for mid
    public static double toTurn = 18;
    public static double backBoard = 24;

    public static double farBoxToPerp = 22;

    //for left side, right and left should be modified to not go backwards, but rather go forward then strafe right
    //for the front face, it should just be regular
    //need the variables from the middle of the "box" to the perpendicular of the parking placement and then distance from that point to placement

    //Add more as needed
    public static Pose2d startPoseCloseRed = new Pose2d(12,-62, Math.toRadians(90)); //This should be close to correct
    public static Pose2d startPoseFarRed = new Pose2d(-35, -62, Math.toRadians(90)); //This too
    public static Pose2d startPoseCloseBlue = new Pose2d(12,62, Math.toRadians(-90));//And this
    public static Pose2d startPoseFarBlue = new Pose2d(-35,62, Math.toRadians(-90)); //Also this
    public static Pose2d buildedCloseRed = new Pose2d(12,-62, Math.toRadians(0));
    public static Pose2d buildedFarRed = new Pose2d(45.5, -28.8, 0);
    public static Pose2d redB4park = new Pose2d(12, -62, 0);

//    public static Pose2d a = new Pose2d(12, -63, 0);
    public static Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 17.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseFarBlue)
                                .lineToLinearHeading(new Pose2d(-35.6, 32, Math.toRadians(-90)))
                                .back(6)
                                .strafeRight(15)
                                .lineToLinearHeading(new Pose2d(-51.6, 7.4, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47.4, 7.4, Math.toRadians(0)))
                                .strafeLeft(25)
                                .lineToLinearHeading(new Pose2d(46.5, 10.9, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, 10.9, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(46.5, 10.9, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(47.2, 29.3, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(46.5, 10.9, 0))
                                .forward(10)
                                .build()



                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
