package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration class exposing live adjustable camera and vision parameters
 * to the FTC Dashboard web interface for tuning the GoBilda Global Shutter Camera
 * during the BioBuzz season.
 */
@Config
public class CameraConfig {
    // Hardware map name for the camera
    public static String CAMERA_NAME = "Webcam 1";

    // Resolution settings (e.g., 640x480, 800x600, 1280x720)
    public static int STREAM_WIDTH = 1280;
    public static int STREAM_HEIGHT = 800;

    // Exposure & Gain settings (Global shutter cameras benefit from short exposures)
    public static boolean AUTO_EXPOSURE = false;
    public static long EXPOSURE_MS = 1;
    public static int GAIN = 100;

    // AprilTag Processor settings
    // DECIMATION controls the downsampling of the camera image to balance detection speed versus tracking distance accuracy
    public static float DECIMATION = 1.0f; // Decimation 1 = max distance accuracy/slower, 2 = fast FPS/good range
    public static boolean DRAW_AXES = true;
    public static boolean DRAW_TAG_OUTLINE = true;
    public static boolean DRAW_CUBE_PROJECTION = false;
    public static boolean DRAW_TAG_ID = true;

    // Filter by specific cluster name ("ALL", "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING")
    public static String CLUSTER_FILTER = "ALL";

    // Private constructor to prevent instantiation
    private CameraConfig() {}

    // --- Camera Lens Intrinsics (640x480) ---
    // TODO: Replace these placeholders with the values generated from 3DF Zephyr
    // These numbers were initialized with the numbers from the GoBilda Global Shutter User Guide
    // These values are probably not needed because GoBilda also gave us a teamwebcamcalibrations.xml file
    public static final double FX = 545.584f;
    public static final double FY = 545.584f;
    public static final double CX = 350.588f;
    public static final double CY = 223.150f;

    // --- Field Dimensions ---
    private static final double HALF_FIELD_INCHES = 72.0;

    /**
     * Converts native FTC SDK AprilTag pose coordinates to PedroPathing coordinates.
     *
     * FTC Origin (0,0): Center of the field. +X faces standard forward, +Y faces left.
     * Pedro Origin (0,0): Bottom-left corner of the field. +X faces forward, +Y faces left.
     *
     * @param ftcX The X coordinate provided by the FTC AprilTag pose (in inches).
     * @param ftcY The Y coordinate provided by the FTC AprilTag pose (in inches).
     * @param ftcHeading The heading provided by the FTC AprilTag pose (in radians).
     * @return A double array containing [pedroX, pedroY, pedroHeading] in inches and radians.
     */
    public static double[] toPedroCoordinates(double ftcX, double ftcY, double ftcHeading) {
        // Shift the origin from the center of the field to the bottom-left corner
        // and map the corresponding rotated vector components.
        double pedroX = ftcY + HALF_FIELD_INCHES;
        double pedroY = HALF_FIELD_INCHES - ftcX;

        // Rotate the heading coordinate system to line up with PedroPathing's 0-radian baseline
        double pedroHeading = ftcHeading - (Math.PI / 2.0);

        // Normalize heading to stay within the traditional [-PI, PI] range
        while (pedroHeading > Math.PI)   pedroHeading -= 2 * Math.PI;
        while (pedroHeading <= -Math.PI) pedroHeading += 2 * Math.PI;

        return new double[]{pedroX, pedroY, pedroHeading};
    }

}
