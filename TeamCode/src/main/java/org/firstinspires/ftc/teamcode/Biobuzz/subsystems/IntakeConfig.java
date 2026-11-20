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
    public static String DEPOLLINATOR_SERVO_NAME = "depollinatorServo";

    // Motor Target Velocity Constants (encoder ticks per second)
    public static double DEFAULT_INTAKE_VELOCITY = 1110.0;
    public static boolean INTAKE_MOTOR_REVERSED = true;

    // De-pollinator Servo Position Constants
    public static double DEPOLLINATOR_SERVO_UP = 0.5; //TODO - test and determine these values, setting around the middle to minimize safety concerns
    public static double DEPOLLINATOR_SERVO_DOWN = 0.0;

    // Private constructor prevents instantiation
    private IntakeConfig() {}
}
