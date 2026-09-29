package org.firstinspires.ftc.teamcode.summerChassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "Dual Vision: Limelight + Webcam", group = "Sensor")
public class limelight_webcam extends LinearOpMode {

    // VisionPortal / Webcam objects
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Limelight object
    private Limelight3A limelight;



    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize Webcam + AprilTag ---
        initAprilTag();  // calling the initAprrilTag;   this is for the webcam.

        // --- Initialize Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Dual Vision Ready. Press PLAY to begin...");
        telemetry.update();
        waitForStart();

        //so the plan is to call the limelight in the opModeIsActive loop itself, and then make method for the webcam telemetry and then call is in the while opModeIsActive there

        while (opModeIsActive()) {
            // --- Webcam Telemetry ---
            telemetryAprilTag();

            // --- Limelight Telemetry ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                telemetry.addData("Limelight Tags Detected", fiducialResults.size());

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    double targetOffsetAngle_Vertical = fr.getTargetYDegrees();
                    double limelightMountAngleDegrees = 0;
                    double limelightLensHeightInches = 5.625;
                    double goalHeightInches = 29.5;

                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
                    double distanceInches = (goalHeightInches - limelightLensHeightInches) / Math.sin(angleToGoalRadians);

                    telemetry.addLine(String.format("LL Tag ID: %d, Family: %s",
                            fr.getFiducialId(), fr.getFamily()));
                    telemetry.addData("→ Offsets", "X=%.2f°, Y=%.2f°", fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    telemetry.addData("→ Distance", "%.1f in", distanceInches);
                }
            } else {
                telemetry.addLine("Limelight: No target detected");
            }

            telemetry.update();
            sleep(20);
        }

        // Shutdown
        visionPortal.close();
        limelight.stop();




    }

    // Initialize the AprilTag processor and VisionPortal for the webcam.
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();




    }

    // Show telemetry for AprilTag detections from the webcam.
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Webcam Tags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }
}
