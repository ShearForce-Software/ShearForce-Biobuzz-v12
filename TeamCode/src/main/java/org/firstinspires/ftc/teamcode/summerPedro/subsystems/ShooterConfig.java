package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;

/*
This class exposes static variables to the FTC Dashboard web server (http://192.168.43.1:8080/dash).
Adding the @Config annotation makes these values fully interactive while the robot is running.

Calculation for goBilda 5203 Series 1:1 Yellow Jacket Motors:
- Bare Motor Max RPM: 6000 RPM
- Gear Ratio: 1:1
- Encoder CPR: 28 ticks per revolution
- Target 2378.57 RPM: (2378.57 RPM / 60 sec) * 28 ticks/rev = 1110 ticks/sec
- Predicted Feedforward F: 32767 / Target_Ticks_Per_Sec = 32767 / ((2378.57/60) * 28) = 32767 / 1110 = 29.52
 */
@Config
public class ShooterConfig {
    // Exposing variables as public static lets Dashboard edit them live
    public static double MOTOR_ENCODER_TICKS_PER_REVOLUTION = 28.0; // goBilda 1:1 Yellow Jacket (28 CPR)

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 29.52;
    public static double VELOCITY_TOLERANCE = 25.0; // ~53 RPM tolerance

    public static double SHOOTER_SERVO_REST = 1.0;
    public static double SHOOTER_SERVO_SHOOT = 0.8;

    public static double DEFAULT_HIGH_VELOCITY = 1110.0;
    public static double DEFAULT_LOW_VELOCITY = 500.0;

    // Private constructor prevents instantiation
    private ShooterConfig() {}

}
