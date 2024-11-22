package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double X_OFFSET = 0.984252; // DO NOT CHANGE
    public static double Y_OFFSET = 0.629921; // DO NOT CHANGE
    public static double MASS_IN_KG = 11.6; //11.6 kg = ~25.57 lbs
    public static double X_MOVEMENT = 61.0745917224;
    public static double Y_MOVEMENT = 46.819712548;
    public static double A_SCALER = 0.97546;
    public static double L_SCALER = 1.02870762962;
    public static double forwardZeroPowerAccel = -55.434400694;
    public static double lateralZeroPowerAccel = -95.2948464538;
    public static double zeroPowerAccelMultiplier = 4;
    public static double centripetalForceScaling = 0.0007;
    public static double translationalP = 0.26, translationalI = 0, translationalD = 0.02, translationalF = 0,
            driveP = 0.005, driveI = 0, driveD = 0.0005, driveT = 0.6, driveF = 0; // Might need to be changed
}
