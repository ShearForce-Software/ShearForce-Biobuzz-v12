package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.GoBildaCameraConfig;

/**
 * Interactive Test and Tuning OpMode for the GoBilda Global Shutter Camera.
 * 
 * Features:
 * - BioBuzz Season AprilTag Cluster Detection and Pose Tracking (RED SCORING, RED AUDIENCE,
 *   BLUE AUDIENCE, BLUE SCORING).
 * - Real-time camera setting adjustments (Exposure, Gain, Decimation, Auto/Manual Exposure)
 *   via Gamepad 1 controls and FTC Dashboard.
 * - Live camera FPS and camera controls telemetry.
 * 
 * Gamepad 1 Controls:
 * - D-Pad Up / Down: Increase / Decrease Exposure (ms)
 * - D-Pad Left / Right: Decrease / Increase Gain
 * - Left Bumper / Right Bumper: Decrease / Increase Decimation (1.0, 1.5, 2.0, 3.0, 4.0)
 * - Button Cross (A): Toggle Auto vs Manual Exposure
 * - Button Square (X): Cycle Cluster Filter ("ALL", "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING")
 * - Button Triangle (Y): Reset Camera Settings to Defaults
 */
@TeleOp(name = "GoBilda Camera AprilTag Tuner", group = "summerPedro")
public class GoBildaCameraTunerOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Camera control interfaces
    private ExposureControl exposureControl;
    private GainControl gainControl;

    // Hardware limits
    private long minExposureMs = 1;
    private long maxExposureMs = 100;
    private int minGain = 0;
    private int maxGain = 255;

    // Local tuning variables initialized from Config
    private boolean autoExposure = GoBildaCameraConfig.AUTO_EXPOSURE;
    private long exposureMs = GoBildaCameraConfig.EXPOSURE_MS;
    private int gain = GoBildaCameraConfig.GAIN;
    private float decimation = GoBildaCameraConfig.DECIMATION;
    private int clusterFilterIndex = 0;

    private static final String[] CLUSTER_FILTERS = {
            "ALL", "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"
    };

    // Button edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean lastTriangle = false;

    private static final float[] DECIMATION_STEPS = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f};

    @Override
    public void runOpMode() throws InterruptedException {
        // Multi-telemetry pushes logs to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== Initializing GoBilda Global Shutter Camera ===");
        telemetry.update();

        initVisionPortal();

        // Wait for camera stream to start before querying control bounds
        waitForCameraStreaming();

        queryCameraControlBounds();

        // Apply initial settings
        applyCameraSettings();

        telemetry.addLine("=== Camera Ready ===");
        telemetry.addLine("Press START to run interactive tuning.");
        telemetry.update();

        waitForStart();

        // Sync values from dashboard
        FtcDashboard.getInstance().updateConfig();

        while (opModeIsActive()) {
            // Process driver gamepad inputs for tuning
            handleGamepadInput();

            // Sync with Dashboard config values if modified via browser
            syncDashboardConfig();

            // Display detections, cluster tracking, and camera performance telemetry
            displayTelemetry();

            telemetry.update();
            sleep(20);
        }

        // Clean up camera stream on stop
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initializes the AprilTagProcessor and VisionPortal using the BioBuzz Tag Library.
     */
    private void initVisionPortal() {
        // Build AprilTag Processor using the full current game tag library (includes BioBuzz clusters)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(GoBildaCameraConfig.DRAW_AXES)
                .setDrawTagOutline(GoBildaCameraConfig.DRAW_TAG_OUTLINE)
                .setDrawCubeProjection(GoBildaCameraConfig.DRAW_CUBE_PROJECTION)
                .setDrawTagID(GoBildaCameraConfig.DRAW_TAG_ID)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTagProcessor.setDecimation(decimation);

        WebcamName webcamName;
        try {
            webcamName = hardwareMap.get(WebcamName.class, GoBildaCameraConfig.CAMERA_NAME);
        } catch (Exception e) {
            telemetry.addData("Warning", "Camera '%s' not found, trying 'Webcam 1'", GoBildaCameraConfig.CAMERA_NAME);
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(GoBildaCameraConfig.STREAM_WIDTH, GoBildaCameraConfig.STREAM_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Waits until the VisionPortal camera stream is active.
     */
    private void waitForCameraStreaming() {
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Queries min/max limits for Exposure and Gain supported by the GoBilda camera.
     */
    private void queryCameraControlBounds() {
        if (visionPortal == null) return;

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            minExposureMs = Math.max(1, exposureControl.getMinExposure(TimeUnit.MILLISECONDS));
            maxExposureMs = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        }

        gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }

    /**
     * Applies exposure, gain, and decimation settings to the camera and processor.
     */
    private void applyCameraSettings() {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        if (exposureControl != null) {
            if (autoExposure) {
                if (exposureControl.isModeSupported(ExposureControl.Mode.ContinuousAuto)) {
                    exposureControl.setMode(ExposureControl.Mode.ContinuousAuto);
                }
            } else {
                if (exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                exposureMs = Math.max(minExposureMs, Math.min(maxExposureMs, exposureMs));
                exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            }
        }

        if (gainControl != null && !autoExposure) {
            gain = Range.clip(gain, minGain, maxGain);
            gainControl.setGain(gain);
        }

        if (aprilTagProcessor != null) {
            aprilTagProcessor.setDecimation(decimation);
        }
    }

    /**
     * Handles Gamepad 1 button inputs for live tuning during execution.
     */
    private void handleGamepadInput() {
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;
        boolean currentDpadLeft = gamepad1.dpad_left;
        boolean currentDpadRight = gamepad1.dpad_right;
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentRightBumper = gamepad1.right_bumper;
        boolean currentCross = gamepad1.cross || gamepad1.a;
        boolean currentSquare = gamepad1.square || gamepad1.x;
        boolean currentTriangle = gamepad1.triangle || gamepad1.y;

        long exposureStep = gamepad1.right_trigger > 0.3 ? 5 : 1;
        int gainStep = gamepad1.right_trigger > 0.3 ? 20 : 5;

        boolean changed = false;

        // D-Pad Up / Down: Adjust Exposure
        if (currentDpadUp && !lastDpadUp) {
            autoExposure = false;
            exposureMs += exposureStep;
            changed = true;
        } else if (currentDpadDown && !lastDpadDown) {
            autoExposure = false;
            exposureMs -= exposureStep;
            changed = true;
        }

        // D-Pad Left / Right: Adjust Gain
        if (currentDpadRight && !lastDpadRight) {
            autoExposure = false;
            gain += gainStep;
            changed = true;
        } else if (currentDpadLeft && !lastDpadLeft) {
            autoExposure = false;
            gain -= gainStep;
            changed = true;
        }

        // Bumpers: Adjust Decimation step
        if (currentRightBumper && !lastRightBumper) {
            decimation = getNextDecimation(decimation, 1);
            changed = true;
        } else if (currentLeftBumper && !lastLeftBumper) {
            decimation = getNextDecimation(decimation, -1);
            changed = true;
        }

        // Button Cross (A): Toggle Auto vs Manual Exposure
        if (currentCross && !lastCross) {
            autoExposure = !autoExposure;
            changed = true;
        }

        // Button Square (X): Cycle Cluster Filter
        if (currentSquare && !lastSquare) {
            clusterFilterIndex = (clusterFilterIndex + 1) % CLUSTER_FILTERS.length;
            GoBildaCameraConfig.CLUSTER_FILTER = CLUSTER_FILTERS[clusterFilterIndex];
        }

        // Button Triangle (Y): Reset settings to defaults
        if (currentTriangle && !lastTriangle) {
            autoExposure = false;
            exposureMs = GoBildaCameraConfig.EXPOSURE_MS;
            gain = GoBildaCameraConfig.GAIN;
            decimation = GoBildaCameraConfig.DECIMATION;
            changed = true;
        }

        lastDpadUp = currentDpadUp;
        lastDpadDown = currentDpadDown;
        lastDpadLeft = currentDpadLeft;
        lastDpadRight = currentDpadRight;
        lastLeftBumper = currentLeftBumper;
        lastRightBumper = currentRightBumper;
        lastCross = currentCross;
        lastSquare = currentSquare;
        lastTriangle = currentTriangle;

        if (changed) {
            applyCameraSettings();
            // Sync local values back to Config for Dashboard display
            GoBildaCameraConfig.AUTO_EXPOSURE = autoExposure;
            GoBildaCameraConfig.EXPOSURE_MS = exposureMs;
            GoBildaCameraConfig.GAIN = gain;
            GoBildaCameraConfig.DECIMATION = decimation;
        }
    }

    private float getNextDecimation(float current, int dir) {
        int idx = 2; // Default to 2.0f
        float minDiff = Float.MAX_VALUE;
        for (int i = 0; i < DECIMATION_STEPS.length; i++) {
            float diff = Math.abs(DECIMATION_STEPS[i] - current);
            if (diff < minDiff) {
                minDiff = diff;
                idx = i;
            }
        }
        idx = Range.clip(idx + dir, 0, DECIMATION_STEPS.length - 1);
        return DECIMATION_STEPS[idx];
    }

    /**
     * Reads changes made from FTC Dashboard config web client.
     */
    private void syncDashboardConfig() {
        boolean changed = false;
        if (GoBildaCameraConfig.AUTO_EXPOSURE != autoExposure) {
            autoExposure = GoBildaCameraConfig.AUTO_EXPOSURE;
            changed = true;
        }
        if (GoBildaCameraConfig.EXPOSURE_MS != exposureMs) {
            exposureMs = GoBildaCameraConfig.EXPOSURE_MS;
            changed = true;
        }
        if (GoBildaCameraConfig.GAIN != gain) {
            gain = GoBildaCameraConfig.GAIN;
            changed = true;
        }
        if (Math.abs(GoBildaCameraConfig.DECIMATION - decimation) > 0.05f) {
            decimation = GoBildaCameraConfig.DECIMATION;
            changed = true;
        }
        if (changed) {
            applyCameraSettings();
        }
    }

    /**
     * Renders telemetry for camera controls, performance, and detected BioBuzz AprilTag clusters.
     */
    private void displayTelemetry() {
        telemetry.addLine("=== GoBilda Global Shutter Camera Tuner ===");
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Camera FPS", String.format(Locale.US, "%.1f FPS", visionPortal.getFps()));
        telemetry.addData("Exposure Mode", autoExposure ? "AUTO" : "MANUAL");
        telemetry.addData("Exposure (ms)", String.format(Locale.US, "%d ms (Limits: %d - %d ms)", exposureMs, minExposureMs, maxExposureMs));
        telemetry.addData("Gain", String.format(Locale.US, "%d (Limits: %d - %d)", gain, minGain, maxGain));
        telemetry.addData("Decimation", String.format(Locale.US, "%.1f", decimation));
        telemetry.addData("Cluster Filter", CLUSTER_FILTERS[clusterFilterIndex]);
        telemetry.addLine();

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        int totalDetections = detections.size();
        telemetry.addData("Total AprilTag Detections", totalDetections);

        int clusterCount = 0;
        int singleCount = 0;

        AprilTagClusterDetection closestCluster = null;
        double minRange = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String clusterName = (cluster.metadata != null) ? cluster.metadata.name : "Unknown Cluster";

                // Filter by cluster selection if active
                String activeFilter = CLUSTER_FILTERS[clusterFilterIndex];
                if (!"ALL".equals(activeFilter) && !clusterName.contains(activeFilter)) {
                    continue;
                }

                clusterCount++;

                telemetry.addLine(String.format(Locale.US,
                        "\n--- Cluster [%s] ---", clusterName));
                telemetry.addLine(String.format(Locale.US,
                        "  Tags Present in Cluster: %d%%", cluster.percentClusterFound));

                if (detection.ftcPose != null) {
                    telemetry.addLine(String.format(Locale.US,
                            "  XYZ (in): X=%.2f, Y=%.2f, Z=%.2f",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format(Locale.US,
                            "  PRY (deg): Pitch=%.1f, Roll=%.1f, Yaw=%.1f",
                            detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format(Locale.US,
                            "  Range/Bearing/Elevation: Range=%.2f in, Bearing=%.1f°, Elev=%.1f°",
                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    if (detection.ftcPose.range < minRange) {
                        minRange = detection.ftcPose.range;
                        closestCluster = cluster;
                    }
                }
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleTag = (AprilTagSingleDetection) detection;
                singleCount++;

                String tagName = (singleTag.metadata != null) ? singleTag.metadata.name : "Tag " + singleTag.id;
                telemetry.addLine(String.format(Locale.US,
                        "Single Tag ID %d [%s] @ (%.1f, %.1f) px",
                        singleTag.id, tagName, singleTag.center.x, singleTag.center.y));
            }
        }

        telemetry.addLine();
        telemetry.addData("Clusters Found", clusterCount);
        telemetry.addData("Single Tags Found", singleCount);

        if (closestCluster != null && closestCluster.metadata != null && closestCluster.ftcPose != null) {
            telemetry.addLine(String.format(Locale.US,
                    "\n>>> TRACKING CLOSEST CLUSTER: %s (%.1f in away) <<<",
                    closestCluster.metadata.name, closestCluster.ftcPose.range));
        }

        telemetry.addLine("\n--- Gamepad 1 Controls ---");
        telemetry.addLine("D-Pad Up/Down: Adj Exposure (+/- 1ms)");
        telemetry.addLine("D-Pad Left/Right: Adj Gain (+/- 5)");
        telemetry.addLine("Hold R-Trigger: Fast Exposure (+5ms) / Gain (+20)");
        telemetry.addLine("Bumpers L/R: Decimation (1.0..4.0)");
        telemetry.addLine("Cross (A): Toggle Auto Exposure");
        telemetry.addLine("Square (X): Cycle Cluster Filter");
        telemetry.addLine("Triangle (Y): Reset Defaults");
    }
}
