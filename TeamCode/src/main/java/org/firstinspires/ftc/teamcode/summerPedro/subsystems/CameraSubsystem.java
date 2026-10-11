package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * CameraSubsystem encapsulating the GoBilda Global Shutter Camera, AprilTag & Cluster tracking,
 * real-time camera controls (Exposure, Gain, Decimation), FTC Dashboard stream integration,
 * and PedroPathing heading drift correction.
 */
public class CameraSubsystem {

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

    // Active camera setting variables
    private boolean autoExposure = CameraConfig.AUTO_EXPOSURE;
    private long exposureMs = CameraConfig.EXPOSURE_MS;
    private int gain = CameraConfig.GAIN;
    private float decimation = CameraConfig.DECIMATION;

    public CameraSubsystem(HardwareMap hardwareMap) {
        initVisionPortal(hardwareMap);
    }

    /**
     * Initializes the AprilTagProcessor and VisionPortal using hardwareMap settings and CameraConfig.
     */
    private void initVisionPortal(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(CameraConfig.DRAW_AXES)
                .setDrawTagOutline(CameraConfig.DRAW_TAG_OUTLINE)
                .setDrawCubeProjection(CameraConfig.DRAW_CUBE_PROJECTION)
                .setDrawTagID(CameraConfig.DRAW_TAG_ID)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTagProcessor.setDecimation(decimation);

        WebcamName webcamName;
        try {
            webcamName = hardwareMap.get(WebcamName.class, CameraConfig.CAMERA_NAME);
        } catch (Exception e) {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(CameraConfig.STREAM_WIDTH, CameraConfig.STREAM_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Queries min/max limits for Exposure and Gain supported by the camera and applies current settings.
     */
    public void queryCameraControlBoundsAndApply() {
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

        applyCameraSettings();
    }

    /**
     * Applies exposure, gain, and decimation settings to the active camera and processor.
     */
    public void applyCameraSettings() {
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
            gain = Math.max(minGain, Math.min(maxGain, gain));
            gainControl.setGain(gain);
        }

        if (aprilTagProcessor != null) {
            aprilTagProcessor.setDecimation(decimation);
        }
    }

    /**
     * Enables streaming camera frames to FTC Dashboard.
     */
    public void startDashboardStream() {
        if (visionPortal != null) {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        }
    }

    /**
     * Stops streaming camera frames to FTC Dashboard.
     */
    public void stopDashboardStream() {
        FtcDashboard.getInstance().stopCameraStream();
    }

    /**
     * Calculates the corrected PedroPathing robot heading (in radians) from an AprilTag cluster detection.
     * Uses CameraConfig.toPedroCoordinates to convert coordinate systems and eliminate heading drift.
     *
     * @param cluster The AprilTagClusterDetection object.
     * @return The corrected Pedro heading in radians, or Double.NaN if the detection or ftcPose is invalid.
     */
    public double getCorrectedPedroHeading(AprilTagClusterDetection cluster) {
        if (cluster == null || cluster.ftcPose == null) {
            return Double.NaN;
        }

        // Convert FTC yaw angle (degrees) to radians
        double ftcHeadingRad = Math.toRadians(cluster.ftcPose.yaw);

        // Convert FTC coordinates to Pedro coordinates using CameraConfig.toPedroCoordinates
        double[] pedroCoords = CameraConfig.toPedroCoordinates(cluster.ftcPose.x, cluster.ftcPose.y, ftcHeadingRad);

        // Index 2 is the converted Pedro heading in radians
        return pedroCoords[2];
    }

    /**
     * Overload that finds the closest valid AprilTag cluster detection and calculates the corrected Pedro heading.
     *
     * @return The corrected Pedro heading in radians, or Double.NaN if no valid cluster is detected.
     */
    public double getCorrectedPedroHeading() {
        AprilTagClusterDetection closestCluster = getClosestClusterDetection();
        return getCorrectedPedroHeading(closestCluster);
    }

    /**
     * Returns the closest AprilTagClusterDetection from current detections matching the active CameraConfig cluster filter.
     */
    public AprilTagClusterDetection getClosestClusterDetection() {
        if (aprilTagProcessor == null) return null;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagClusterDetection closestCluster = null;
        double minRange = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String activeFilter = CameraConfig.CLUSTER_FILTER;
                String clusterName = (cluster.metadata != null) ? cluster.metadata.name : "";

                if (!"ALL".equals(activeFilter) && !clusterName.contains(activeFilter)) {
                    continue;
                }

                if (cluster.ftcPose != null && cluster.ftcPose.range < minRange) {
                    minRange = cluster.ftcPose.range;
                    closestCluster = cluster;
                }
            }
        }
        return closestCluster;
    }

    /**
     * Returns all current AprilTag detections.
     */
    public List<AprilTagDetection> getDetections() {
        return (aprilTagProcessor != null) ? aprilTagProcessor.getDetections() : Collections.emptyList();
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    // --- Control Getters and Setters ---

    public boolean isAutoExposure() {
        return autoExposure;
    }

    public void setAutoExposure(boolean autoExposure) {
        this.autoExposure = autoExposure;
    }

    public long getExposureMs() {
        return exposureMs;
    }

    public void setExposureMs(long exposureMs) {
        this.exposureMs = exposureMs;
    }

    public int getGain() {
        return gain;
    }

    public void setGain(int gain) {
        this.gain = gain;
    }

    public float getDecimation() {
        return decimation;
    }

    public void setDecimation(float decimation) {
        this.decimation = decimation;
    }

    public long getMinExposureMs() {
        return minExposureMs;
    }

    public long getMaxExposureMs() {
        return maxExposureMs;
    }

    public int getMinGain() {
        return minGain;
    }

    public int getMaxGain() {
        return maxGain;
    }

    /**
     * Closes the camera stream and cleans up resources.
     */
    public void close() {
        stopDashboardStream();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
