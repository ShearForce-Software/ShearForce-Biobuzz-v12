package org.firstinspires.ftc.teamcode.Biobuzz.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration class for the IntakeSubsystem.
 * Exposes hardware map device names and tuning parameters to FTC Dashboard (@Config).
 */
@Config
public class IntakeConfig {

    // Hardware Map Device Names
    public static String INTAKE_MOTOR_NAME = "intake";
    public static String COLOR_SENSOR_RAMP_BACK_NAME = "intakeColorRampBack";
    public static String COLOR_SENSOR_RAMP_FRONT_NAME = "intakeColorRampFront";
    public static String DEPOLLINATOR_SERVO_NAME = "depollinatorServo";

    // Motor Target Velocity Constants (encoder ticks per second)
    public static double DEFAULT_INTAKE_VELOCITY = 1110.0;
    public static boolean INTAKE_MOTOR_REVERSED = true;

    // De-pollinator Servo Position Constants
    public static double DEPOLLINATOR_SERVO_UP = 0.5; //TODO - test and determine these values, setting around the middle to minimize safety concerns
    public static double DEPOLLINATOR_SERVO_DOWN = 0.0;

    // Color Sensor Detection Thresholds
    public static double DISTANCE_THRESHOLD_MM = 35.0; // Distance in mm to consider a ball present
    public static float RED_MIN_RATIO = 0.40f;          // Min red/total ratio for Red ball
    public static float BLUE_MIN_RATIO = 0.40f;         // Min blue/total ratio for Blue ball
    public static float YELLOW_RED_MIN_RATIO = 0.30f;   // Min red ratio for Yellow ball
    public static float YELLOW_GREEN_MIN_RATIO = 0.30f; // Min green ratio for Yellow ball

    // Private constructor prevents instantiation
    private IntakeConfig() {}
}
