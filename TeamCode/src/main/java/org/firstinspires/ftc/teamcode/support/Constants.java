package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double X_OFFSET = 0.984252;
    public static double Y_OFFSET = 0.629921;
    public static double MASS_IN_KG = 10.6;
    public static double X_MOVEMENT = 53.66840500469179; // ##.## || Tuned: ##.##
    public static double Y_MOVEMENT = 40.309560390264195; // test values go here, 5 - 10 needed || Tuned: ##.##
    public static double A_SCALER = 0.9828; // ##.## || Tuned: ##.##
    public static double L_SCALER = 0.9744355912359927; // ##.## || Tuned: ##.##
    public static double forwardZeroPowerAccel = -79.3685715955501;
    public static double lateralZeroPowerAccel = -113.73477215002447;
    public static double zeroPowerAccelMultiplier = 2;
    public static double centripetalForceScaling = 0.0005;
    public static double ckP = 0.26, ckI = 0, ckD = 0.02, ckF = 0,
            cfkP = 0.025, cfkI = 0, cfkD = 0.00001, cfkT = 0.6, cfkF = 0;
}
