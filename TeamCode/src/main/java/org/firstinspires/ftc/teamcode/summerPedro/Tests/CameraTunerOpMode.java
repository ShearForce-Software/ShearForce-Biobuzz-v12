package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.CameraConfig;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.CameraSubsystem;

/**
 * Interactive Test and Tuning OpMode for the GoBilda Global Shutter Camera.
 *
 * Uses CameraSubsystem to handle vision setup, settings, and AprilTag cluster tracking.
 *
 * Gamepad 1 Controls:
 * - D-Pad Up / Down: Increase / Decrease Exposure (ms)
 * - D-Pad Left / Right: Decrease / Increase Gain
 * - Left Bumper / Right Bumper: Decrease / Increase Decimation (1.0, 1.5, 2.0, 3.0, 4.0)
 * - Button Cross (A): Toggle Auto vs Manual Exposure
 * - Button Square (X): Cycle Cluster Filter ("ALL", "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING")
 * - Button Triangle (Y): Reset Camera Settings to Defaults
 */
@TeleOp(name = "Camera AprilTag Tuner", group = "Tuning")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class CameraTunerOpMode extends LinearOpMode {

    private CameraSubsystem cameraSubsystem;

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

        telemetry.addLine("=== Initializing GoBilda Global Shutter Camera Subsystem ===");
        telemetry.update();

        cameraSubsystem = new CameraSubsystem(hardwareMap);

        // Wait for camera stream to start before querying control bounds
        waitForCameraStreaming();

        cameraSubsystem.queryCameraControlBoundsAndApply();

        // Enable live camera stream on FTC Dashboard
        cameraSubsystem.startDashboardStream();

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
        if (cameraSubsystem != null) {
            cameraSubsystem.close();
        }
    }

    /**
     * Waits until the VisionPortal camera stream is active.
     */
    private void waitForCameraStreaming() {
        while (!isStopRequested() && cameraSubsystem.getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera State", cameraSubsystem.getVisionPortal().getCameraState());
            telemetry.update();
            sleep(50);
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

        boolean autoExp = cameraSubsystem.isAutoExposure();
        long expMs = cameraSubsystem.getExposureMs();
        int gain = cameraSubsystem.getGain();
        float dec = cameraSubsystem.getDecimation();

        // D-Pad Up / Down: Adjust Exposure
        if (currentDpadUp && !lastDpadUp) {
            autoExp = false;
            expMs += exposureStep;
            changed = true;
        } else if (currentDpadDown && !lastDpadDown) {
            autoExp = false;
            expMs -= exposureStep;
            changed = true;
        }

        // D-Pad Left / Right: Adjust Gain
        if (currentDpadRight && !lastDpadRight) {
            autoExp = false;
            gain += gainStep;
            changed = true;
        } else if (currentDpadLeft && !lastDpadLeft) {
            autoExp = false;
            gain -= gainStep;
            changed = true;
        }

        // Bumpers: Adjust Decimation step
        if (currentRightBumper && !lastRightBumper) {
            dec = getNextDecimation(dec, 1);
            changed = true;
        } else if (currentLeftBumper && !lastLeftBumper) {
            dec = getNextDecimation(dec, -1);
            changed = true;
        }

        // Button Cross (A): Toggle Auto vs Manual Exposure
        if (currentCross && !lastCross) {
            autoExp = !autoExp;
            changed = true;
        }

        // Button Square (X): Cycle Cluster Filter
        if (currentSquare && !lastSquare) {
            clusterFilterIndex = (clusterFilterIndex + 1) % CLUSTER_FILTERS.length;
            CameraConfig.CLUSTER_FILTER = CLUSTER_FILTERS[clusterFilterIndex];
        }

        // Button Triangle (Y): Reset settings to defaults
        if (currentTriangle && !lastTriangle) {
            autoExp = false;
            expMs = CameraConfig.EXPOSURE_MS;
            gain = CameraConfig.GAIN;
            dec = CameraConfig.DECIMATION;
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
            cameraSubsystem.setAutoExposure(autoExp);
            cameraSubsystem.setExposureMs(expMs);
            cameraSubsystem.setGain(gain);
            cameraSubsystem.setDecimation(dec);
            cameraSubsystem.applyCameraSettings();

            // Sync local values back to Config for Dashboard display
            CameraConfig.AUTO_EXPOSURE = autoExp;
            CameraConfig.EXPOSURE_MS = expMs;
            CameraConfig.GAIN = gain;
            CameraConfig.DECIMATION = dec;
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
        if (CameraConfig.AUTO_EXPOSURE != cameraSubsystem.isAutoExposure()) {
            cameraSubsystem.setAutoExposure(CameraConfig.AUTO_EXPOSURE);
            changed = true;
        }
        if (CameraConfig.EXPOSURE_MS != cameraSubsystem.getExposureMs()) {
            cameraSubsystem.setExposureMs(CameraConfig.EXPOSURE_MS);
            changed = true;
        }
        if (CameraConfig.GAIN != cameraSubsystem.getGain()) {
            cameraSubsystem.setGain(CameraConfig.GAIN);
            changed = true;
        }
        if (Math.abs(CameraConfig.DECIMATION - cameraSubsystem.getDecimation()) > 0.05f) {
            cameraSubsystem.setDecimation(CameraConfig.DECIMATION);
            changed = true;
        }
        if (changed) {
            cameraSubsystem.applyCameraSettings();
        }
    }

    /**
     * Renders telemetry for camera controls, performance, and detected BioBuzz AprilTag clusters.
     */
    private void displayTelemetry() {
        VisionPortal vp = cameraSubsystem.getVisionPortal();
        telemetry.addLine("=== GoBilda Global Shutter Camera Tuner ===");
        telemetry.addData("Camera State", vp.getCameraState());
        telemetry.addData("Camera FPS", String.format(Locale.US, "%.1f FPS", vp.getFps()));
        telemetry.addData("Exposure Mode", cameraSubsystem.isAutoExposure() ? "AUTO" : "MANUAL");
        telemetry.addData("Exposure (ms)", String.format(Locale.US, "%d ms (Limits: %d - %d ms)",
                cameraSubsystem.getExposureMs(), cameraSubsystem.getMinExposureMs(), cameraSubsystem.getMaxExposureMs()));
        telemetry.addData("Gain", String.format(Locale.US, "%d (Limits: %d - %d)",
                cameraSubsystem.getGain(), cameraSubsystem.getMinGain(), cameraSubsystem.getMaxGain()));
        telemetry.addData("Decimation", String.format(Locale.US, "%.1f", cameraSubsystem.getDecimation()));
        telemetry.addData("Cluster Filter", CLUSTER_FILTERS[clusterFilterIndex]);

        // Display Heading Correction if available
        double correctedHeading = cameraSubsystem.getCorrectedPedroHeading();
        if (!Double.isNaN(correctedHeading)) {
            telemetry.addData("Corrected Pedro Heading (deg)", String.format(Locale.US, "%.2f°", Math.toDegrees(correctedHeading)));
        } else {
            telemetry.addData("Corrected Pedro Heading", "N/A (No Cluster)");
        }
        telemetry.addLine();

        List<AprilTagDetection> detections = cameraSubsystem.getDetections();
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
