package org.firstinspires.ftc.teamcode.Biobuzz.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration class for the TransferSubsystem.
 * Exposes hardware names and tuning constants to FTC Dashboard (@Config).
 */
@Config
public class TransferConfig {

    // Hardware Map Device Names
    public static String SERVO_NAME = "transferServo";
    public static String FRONT_GATE_SERVO = "frontGateServo";
    public static String ENCODER_NAME = "leftFront_leftOdometry";
    public static String COLOR_SENSOR_TOP_NAME = "transferColorTop";
    public static String COLOR_SENSOR_LEFT_NAME = "transferColorLeft";
    public static String COLOR_SENSOR_RIGHT_NAME = "transferColorRight";
    public static String COLOR_SENSOR_BOTTOM_BACK_NAME = "transferColorBottomBack";
    public static String COLOR_SENSOR_BOTTOM_FRONT_NAME = "transferColorBottomFront";
    public static String COLOR_SENSOR_RAMP_BACK_NAME = "intakeColorRampBack";
    public static String COLOR_SENSOR_RAMP_FRONT_NAME = "intakeColorRampFront";

    // Servo Power Constants
    public static double FORWARD_POWER = -1.0;
    public static double REVERSE_POWER = 0.09;
    public static double STOP_POWER = 0.0;

    // Front Gate Servo Position Constants
    public static double FRONT_GATE_OPEN_POS = 1.0;
    public static double FRONT_GATE_CLOSED_POS = 0.0;

    // REV Through Bore Encoder Constants (8192 ticks per revolution in Quadrature mode)
    public static double TICKS_PER_REV = 8192.0;
    public static double TICKS_PER_SLOT = TICKS_PER_REV / 4.0; // 2048 ticks per 90-degree quadrant
    public static boolean ENCODER_REVERSED = true;

    // Velocity PIDF Gains (Shooting Mode)
    public static double VELOCITY_P = 0.0012;
    public static double VELOCITY_I = 0.0001;
    public static double VELOCITY_D = 0.00005;
    public static double VELOCITY_F = 0.00035; // 1.0 / Max_Velocity_Ticks_Per_Sec

    // Position PID & Predictive Deceleration Gains (Intake Mode)
    public static double POSITION_P = 0.0025;
    public static double POSITION_I = 0.00005;
    public static double POSITION_D = 0.0001;
    public static double MAX_DECEL_TICKS_PER_SEC2 = 6000.0; // Ticks/sec^2 deceleration limit

    // Operating Target Velocities
    public static double SHOOT_VELOCITY_TICKS_PER_SEC = 2800.0;  // Fast forward velocity for shooting
    public static double INTAKE_VELOCITY_TICKS_PER_SEC = -1800.0; // Fast reverse velocity for intaking
    public static double CREEP_VELOCITY_TICKS_PER_SEC = -300.0;   // Slow reverse velocity for final alignment

    // Color Sensor Detection Thresholds (Combined Transfer & Intake Ramp)
    public static double DISTANCE_THRESHOLD_MM = 35.0; // Distance in mm to consider a ball present
    public static float RED_MIN_RATIO = 0.40f;          // Min red/total ratio for Red ball
    public static float BLUE_MIN_RATIO = 0.40f;         // Min blue/total ratio for Blue ball
    public static float YELLOW_RED_MIN_RATIO = 0.30f;   // Min red ratio for Yellow ball
    public static float YELLOW_GREEN_MIN_RATIO = 0.30f; // Min green ratio for Yellow ball

    public static boolean AUTO_TRANSFER = true;

    // Private constructor prevents instantiation
    private TransferConfig() {}
}
