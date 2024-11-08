package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static double kP = 0.0, kI = 0.0, kD = 0.0, f = 0.0;
    public static double armStart = 0.0;
    public static double ARM_TARGET = 0.0;
    public static double ARM_OFF = 0.0;
    public static double ARM_SPEED = 0.0;
    public static double TICK_PER_RAD = (((145.6 * 19.2) * 2) / (2 * Math.PI));
}
