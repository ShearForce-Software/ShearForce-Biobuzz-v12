package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration class for the TransferSubsystem.
 * Exposes hardware names and tuning constants to FTC Dashboard (@Config).
 */
@Config
public class TransferConfig {

    // Hardware Map Device Names
    public static String SERVO_NAME = "transferServo";
    public static String ENCODER_NAME = "transferEncoder";
    public static String COLOR_SENSOR_1_NAME = "transferColor1";
    public static String COLOR_SENSOR_2_NAME = "transferColor2";
    public static String COLOR_SENSOR_3_NAME = "transferColor3";
    public static String COLOR_SENSOR_4_NAME = "transferColor4";

    // Servo Power Constants
    public static double FORWARD_POWER = 0.8;
    public static double REVERSE_POWER = -0.8;
    public static double STOP_POWER = 0.0;

    // REV Through Bore Encoder Constants (8192 ticks per revolution in Quadrature mode)
    public static double TICKS_PER_REV = 8192.0;
    public static boolean ENCODER_REVERSED = false;

    // Color Sensor Detection Thresholds
    public static double DISTANCE_THRESHOLD_MM = 45.0; // Distance in mm to consider a ball present
    public static float RED_MIN_RATIO = 0.40f;          // Min red/total ratio for Red ball
    public static float BLUE_MIN_RATIO = 0.40f;         // Min blue/total ratio for Blue ball
    public static float YELLOW_RED_MIN_RATIO = 0.30f;   // Min red ratio for Yellow ball
    public static float YELLOW_GREEN_MIN_RATIO = 0.30f; // Min green ratio for Yellow ball

    // Private constructor prevents instantiation
    private TransferConfig() {}
}
