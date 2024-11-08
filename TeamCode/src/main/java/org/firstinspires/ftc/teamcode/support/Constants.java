package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double X_OFFSET = 0.984252;
    public static double Y_OFFSET = 0.629921;
    public static double MASS_IN_KG = 10.6;
    public static double X_MOVEMENT = 81.34056; // might be correct, needs to be checked tho -> 60.617520819111576, 58.74611374581603, 60.226880730156, 60.30598867806962,  || Tuned: ##.##
    public static double Y_MOVEMENT = 65.43028; // test values go here, 5 - 10 needed || Tuned: ##.##
    public static double A_SCALER = 0.9854787550744248; // 1.1589716331808235, 1.1453018281895027, 1.1626722011574535, 1.219370113017999, 1.2947000888888889 || Tuned: 1.19620317288693352
    public static double L_SCALER = 0.9794687628985508; // 1.1777138467758237, 1.1777138467758237, 1.1865886761710793 || Tuned: 1.1806721232409089
    public static double forwardZeroPowerAccel = -34.62719;
    public static double lateralZeroPowerAccel = -78.15554;
    public static double centripetalForceScaling = 0.0005;
    public static double ckP = 0.1, ckI = 0, ckD = 0, ckF = 0,
            cfkP = 0.025, cfkI = 0, cfkD = 0.00001, cfkT = 0.6, cfkF = 0;
}
