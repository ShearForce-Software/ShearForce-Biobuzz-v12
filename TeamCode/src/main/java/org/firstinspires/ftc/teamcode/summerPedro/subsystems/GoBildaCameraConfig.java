package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration class exposing live adjustable camera and vision parameters
 * to the FTC Dashboard web interface for tuning the GoBilda Global Shutter Camera
 * during the BioBuzz season.
 */
@Config
public class GoBildaCameraConfig {
    // Hardware map name for the camera
    public static String CAMERA_NAME = "Webcam 1";

    // Resolution settings (e.g., 640x480, 800x600, 1280x720)
    public static int STREAM_WIDTH = 640;
    public static int STREAM_HEIGHT = 480;

    // Exposure & Gain settings (Global shutter cameras benefit from short exposures)
    public static boolean AUTO_EXPOSURE = false;
    public static long EXPOSURE_MS = 1;
    public static int GAIN = 100;

    // AprilTag Processor settings
    public static float DECIMATION = 2.0f; // Decimation 1 = max range/slower, 2 = fast FPS/good range
    public static boolean DRAW_AXES = true;
    public static boolean DRAW_TAG_OUTLINE = true;
    public static boolean DRAW_CUBE_PROJECTION = false;
    public static boolean DRAW_TAG_ID = true;

    // Filter by specific cluster name ("ALL", "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING")
    public static String CLUSTER_FILTER = "ALL";

    // Private constructor to prevent instantiation
    private GoBildaCameraConfig() {}
}
