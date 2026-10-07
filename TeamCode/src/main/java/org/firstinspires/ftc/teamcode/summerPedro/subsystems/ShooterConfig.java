package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;

/*
This class exposes static variables to the FTC Dashboard web server (http://192.168.43.1:8080/dash).
Adding the @Config annotation makes these values fully interactive while the robot is running.

Calculation for goBilda 5203 Series 1:1 Yellow Jacket Motors:
- Bare Motor Max RPM: 6000 RPM
- Gear Ratio: 1:1
- Encoder CPR: 28 ticks per revolution
- Target 1750 RPM: (1750 RPM / 60 sec) * 28 ticks/rev = 816.67 ticks/sec (~816.7 ticks/sec)
- Theoretical Feedforward F: 32767 / Max_Ticks_Per_Sec = 32767 / ((6000/60) * 28) = 32767 / 2800 = 11.7
 */
@Config
public class ShooterConfig {
    // Exposing variables as public static lets Dashboard edit them live
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 11.7;
    public static double VELOCITY_TOLERANCE = 25.0; // ~53 RPM tolerance

    // Private constructor prevents instantiation
    private ShooterConfig() {}

}
