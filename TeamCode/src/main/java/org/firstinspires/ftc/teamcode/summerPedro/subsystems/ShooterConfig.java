package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;

/*
This class exposes static variables to the FTC Dashboard web server (http://192.168.43).
Adding the @Config annotation makes these values fully interactive while the robot is running.
 */
@Config
public class ShooterConfig {
    // Exposing variables as public static lets Dashboard edit them live
    public static double P = 9.5;
    public static double I = 0.0;
    public static double D = 1.5;
    public static double F = 12.5;

    public static double TARGET_VELOCITY_TICKS = 2100.0;
    public static double VELOCITY_TOLERANCE = 50.0;
}