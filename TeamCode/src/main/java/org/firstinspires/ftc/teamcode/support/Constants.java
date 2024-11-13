package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double X_OFFSET = 0.984252;// Leave This
    public static double Y_OFFSET = 0.629921; // Leave This
    public static double MASS_IN_KG = 11.6; // Leave This MIGHT BE WRONG
    public static double X_MOVEMENT = 61.0745917224; // ##.## || Tuned: ##.##
    public static double Y_MOVEMENT = 46.819712548; // test values go here, 5 - 10 needed || Tuned: ##.##
    public static double A_SCALER = 0.97546; // ##.## || Tuned: ##.##
    public static double L_SCALER = 1.02870762962; // ##.## || Tuned: ##.##
    public static double forwardZeroPowerAccel = -55.434400694;
    public static double lateralZeroPowerAccel = -95.2948464538; //MIGHT BE WRONG BUT I GIVE UP
    public static double zeroPowerAccelMultiplier = 4;// everthing above is tuned, zero power is not
    public static double centripetalForceScaling = 0.0007;
    public static double translationalP = 0.26, translationalI = 0, translationalD = 0.02, translationalF = 0,
            driveP = 0.005, driveI = 0, driveD = 0.0005, driveT = 0.6, driveF = 0; // Might need to be changed
}
