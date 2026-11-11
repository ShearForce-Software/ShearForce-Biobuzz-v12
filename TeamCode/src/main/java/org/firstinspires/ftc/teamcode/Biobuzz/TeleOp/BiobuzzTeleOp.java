package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.Locale;

/**
 * Main TeleOp OpMode for the BioBuzz robot.
 *
 * Gamepad 1 (Driver Controls):
 * - Left Stick: Field-centric Forward / Strafe
 * - Right Stick: Rotation
 * - Button A / Cross: Toggle Position Hold Mode
 * - Button B / Circle: Toggle AprilTag Vision Heading Drift Correction Mode ON/OFF
 *
 * Gamepad 2 (Operator Mechanism Controls):
 * - Button Y / Triangle: Toggle Flywheel Motors ON / OFF
 * - Button A / Cross OR Right Trigger: Shoot (Actuate pusher servo when ready)
 * - Button X / Square: Toggle Intake Motor ON / OFF
 * - Left Trigger / Left Bumper: Reverse / Eject Intake
 * - D-Pad Up / Down: Set Depollinator Servo UP / DOWN
 * - Right Bumper: Advance Transfer 1 Slot (1/4 Rotation)
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class BiobuzzTeleOp extends LinearOpMode {

    // Subsystems
    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ShooterSubsystem shooter;
    private CameraSubsystem camera;

    // Driver States
    private Command holdCommand = null;
    private boolean isHolding = false;
    private boolean visionHeadingCorrectionEnabled = true;

    // Operator States
    private boolean intakeOn = false;

        // Pre-spool flywheels for TeleOp readiness as requested
        //shooter.startFlywheels();


    @Override
    public void runOpMode() throws InterruptedException {

        Scheduler.reset();

        // Instantiate Subsystems
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
        //camera.queryCameraControlBoundsAndApply();

        // Restore exact ending coordinate Pose from Autonomous
        follower.setPose(MatchConfig.currentPose);
        intake.depollinatorUp();
        shooter.hoodMinPosition();
        // =========================================================================
        // GAMEPAD 1: DRIVER CONTROLS
        // =========================================================================
        waitForStart();

        while (opModeIsActive()) {

            // Joystick inputs (invert y-axis for FTC standard forward = negative)
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Break out of position hold if significant driver input is detected
            boolean driverInputDetected = Math.hypot(forward, strafe) > 0.05 || Math.abs(turn) > 0.05;
            if (driverInputDetected && isHolding) {
                isHolding = false;
                if (holdCommand != null) {
                    holdCommand.cancel();
                    holdCommand = null;
                }
            }

            // Toggle Hold Mode via Gamepad 1 Button A (Cross)
            if (gamepad1.aWasPressed() || gamepad1.crossWasPressed()) {
                isHolding = !isHolding;
                if (isHolding) {
                    holdCommand = hold(follower);
                    schedule(holdCommand);
                } else {
                    if (holdCommand != null) {
                        holdCommand.cancel();
                        holdCommand = null;
                    }
                }
            }

            // Toggle Vision Heading Drift Correction Mode via Gamepad 1 Button B (Circle)
            if (gamepad1.bWasPressed() || gamepad1.circleWasPressed()) {
                visionHeadingCorrectionEnabled = !visionHeadingCorrectionEnabled;
            }

            // Handle driving modes
            if (isHolding) {
                Scheduler.execute();
            } else {
                // Field-centric vector calculation
                double robotHeading = follower.pose().heading();
                final double cos = Math.cos(-robotHeading);
                final double sin = Math.sin(-robotHeading);
                double fieldCentricForward = forward * cos - strafe * sin;
                double fieldCentricStrafe = forward * sin + strafe * cos;

                follower.manual(fieldCentricForward, fieldCentricStrafe, turn);
            }

            // Periodically correct heading drift via AprilTag vision when robot is stationary AND mode is enabled
            Velocity robotVel = follower.localizer.velocity();
            double linearSpeed = Math.hypot(robotVel.vx, robotVel.vy);
            double angularSpeed = Math.abs(robotVel.omega);
            boolean isRobotStationary = !driverInputDetected && linearSpeed < 0.05 && angularSpeed < 0.05;

            if (visionHeadingCorrectionEnabled && isRobotStationary) {
                double correctedHeading = camera.getCorrectedPedroHeading();
                if (!Double.isNaN(correctedHeading)) {
                    follower.setHeading(correctedHeading);
                }
            }

            // =========================================================================
            // GAMEPAD 2: OPERATOR MECHANISM CONTROLS
            // =========================================================================

            // 1. Shooter Flywheels Toggle (Button Y / Triangle)
            if (gamepad2.yWasPressed() || gamepad2.triangleWasPressed()) {
                if (shooter.isRunning()) {
                    shooter.stopFlywheels();
                } else {
                    shooter.startFlywheels();
                }
            }

            // 2. Shoot Command (Button A / Cross OR Right Trigger)
            if (gamepad2.a || gamepad2.cross || gamepad2.right_trigger > 0.1) {
                shooter.firePosition();
            } else {
                shooter.restPosition();
            }

            // 3. Intake On/Off Toggle (Button X / Square)
            if (gamepad2.cross || gamepad2.right_trigger > 0.1) {
                intake.turnOn();
            } else if (gamepad2.square || gamepad2.left_trigger > 0.1) {
                intake.reverse();
            } else if (gamepad2.circle) {
                intake.turnOff();
            }
            /*
            if (gamepad2.xWasPressed() || gamepad2.squareWasPressed()) {
                intakeOn = !intakeOn;
                if (intakeOn == true) {
                    intake.turnOn();
                }
                else {intake.turnOff();

                }
            }


            // Intake Motor Execution (Eject takes priority over Intake On)
            if (gamepad2.left_trigger > 0.1 || gamepad2.leftBumperWasPressed()) {
                intake.reverse();
            } else if (intakeOn) {
                intake.turnOn();
            } else {
                intake.turnOff();
            }
            */


            // 4. Depollinator Servo UP / DOWN
            if (gamepad2.dpadUpWasPressed()) {
                intake.depollinatorUp();
            } else if (gamepad2.dpadDownWasPressed()) {
                intake.depollinatorDown();
            }

            // 5. Transfer Advance 1 Slot Forward (Right Bumper)
            if (gamepad2.rightBumperWasPressed()) {
                transfer.advance1SlotReverse();

            } else if (gamepad2.leftBumperWasPressed()) {
               transfer.advance1SlotForward();
            }


        // Update subsystem state loops
            follower.update();
            shooter.update();
            transfer.update();

            // =========================================================================
            // TELEMETRY OUTPUTS
            // =========================================================================

            // --- Match & Target Info ---
            Pose goalPose = MatchConfig.getAudienceGoalPose();
            Pose robotPose = follower.pose();
            double deltaX = goalPose.x() - robotPose.x();
            double deltaY = goalPose.y() - robotPose.y();
            double odomDistance = Math.hypot(deltaX, deltaY);
            double odomBearingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(deltaY, deltaX) - robotPose.heading()));

            AprilTagClusterDetection closestCluster = camera.getClosestClusterDetection();
            double cameraDistance = (closestCluster != null && closestCluster.ftcPose != null) ? closestCluster.ftcPose.range : Double.NaN;
            double cameraBearing = (closestCluster != null && closestCluster.ftcPose != null) ? closestCluster.ftcPose.bearing : Double.NaN;

            telemetry.addLine("=== Match & Target Info ===");
            telemetry.addData("Alliance Color", MatchConfig.allianceColor);
            telemetry.addData("Hive State", MatchConfig.isAudienceGoalUp() ? "AUDIENCE GOAL UP" : "AUDIENCE GOAL DOWN");
            telemetry.addData("Vision Heading Correction", visionHeadingCorrectionEnabled ? "ENABLED (G1:Circle to toggle)" : "DISABLED");
            telemetry.addData("Robot Pose", String.format(Locale.US, "(%.1f, %.1f) %.1f°", robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
            telemetry.addData("Distance to Goal", String.format(Locale.US, "Odom: %.1f in | Camera: %s",
                    odomDistance, Double.isNaN(cameraDistance) ? "N/A" : String.format(Locale.US, "%.1f in", cameraDistance)));
            telemetry.addData("Bearing to Goal", String.format(Locale.US, "Odom: %.1f° | Camera: %s",
                    odomBearingDeg, Double.isNaN(cameraBearing) ? "N/A" : String.format(Locale.US, "%.1f°", cameraBearing)));

            // --- Subsystems Key Data (Velocities & Transfer Servo Position) ---
            telemetry.addLine("=== Subsystems Key Data ===");
            telemetry.addData("Shooter Flywheels", String.format(Locale.US, "Status: %s | Target: %.0f | Actual: %.1f ticks/s",
                    shooter.isRunning() ? "RUNNING" : "STOPPED", shooter.getTargetVelocity(), shooter.getAverageVelocity()));
            telemetry.addData("Intake Motor", String.format(Locale.US, "Status: %s | Target: %.0f | Actual: %.1f ticks/s",
                    intakeOn ? "ON" : "OFF", intake.targetVelocityTicks, intake.getVelocity()));
            telemetry.addData("Transfer Position", String.format(Locale.US, "Power: %.2f | Raw Ticks: %.0f | Rotations: %.3f rev | Angle: %.1f°",
                    transfer.getPower(), transfer.getRawTicks(), transfer.getRotations(), transfer.getDegrees()));
            telemetry.addData("Transfer Velocity", String.format(Locale.US, "%.1f ticks/s (%.1f RPM)",
                    transfer.getVelocityTicksPerSec(), transfer.getVelocityRPM()));

            // --- Controlled Ball Totals (Pollen vs Nectar) & Sensor Readings ---
            int totalBalls = 0;
            int totalPollen = 0;
            int totalNectar = 0;

            TransferSubsystem.BallColor[] slotColors = transfer.getSlotColors();
            for (TransferSubsystem.BallColor color : slotColors) {
                if (color == TransferSubsystem.BallColor.YELLOW) {
                    totalPollen++;
                    totalBalls++;
                } else if (color == TransferSubsystem.BallColor.RED || color == TransferSubsystem.BallColor.BLUE) {
                    totalNectar++;
                    totalBalls++;
                } else if (color != TransferSubsystem.BallColor.NONE && color != TransferSubsystem.BallColor.UNKNOWN) {
                    totalBalls++;
                }
            }

            telemetry.addLine("=== Color & Distance Sensors ===");
            telemetry.addData("Controlled Elements", String.format(Locale.US, "Total Balls: %d/4 | Pollen (Yellow): %d | Nectar (Red/Blue): %d",
                    totalBalls, totalPollen, totalNectar));

            telemetry.addData("Intake Ramp Back", String.format(Locale.US, "Present: %s | Color: %s",
                    intake.isRampBackBallPresent() ? "YES" : "NO ", intake.detectRampBackColor()));
            telemetry.addData("Intake Ramp Front", String.format(Locale.US, "Present: %s | Color: %s",
                    intake.isRampFrontBallPresent() ? "YES" : "NO ", intake.detectRampFrontColor()));

            telemetry.addData("Transfer Top Slot", String.format(Locale.US, "Present: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_TOP) ? "YES" : "NO ", transfer.detectSlotColor(TransferSubsystem.SENSOR_TOP)));
            telemetry.addData("Transfer Left Slot", String.format(Locale.US, "Present: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_LEFT) ? "YES" : "NO ", transfer.detectSlotColor(TransferSubsystem.SENSOR_LEFT)));
            telemetry.addData("Transfer Right Slot", String.format(Locale.US, "Present: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_RIGHT) ? "YES" : "NO ", transfer.detectSlotColor(TransferSubsystem.SENSOR_RIGHT)));
            telemetry.addData("Transfer Bottom Back Slot", String.format(Locale.US, "Present: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_BACK) ? "YES" : "NO ", transfer.detectSlotColor(TransferSubsystem.SENSOR_BOTTOM_BACK)));

            telemetry.addData("Transfer Bottom Front Slot", String.format(Locale.US, "Present: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_FRONT) ? "YES" : "NO ", transfer.detectSlotColor(TransferSubsystem.SENSOR_BOTTOM_FRONT)));

            // --- Controls Summary ---
            telemetry.addLine("=== Controls Summary ===");
            telemetry.addLine("G1 (Driver): Joysticks = Field Drive | Cross (A) = Hold Mode | Circle (B) = Vision Correction");
            telemetry.addLine("G2 (Operator): Triangle (Y) = Flywheels | RT/Cross = Shoot | Square (X) = Intake | LT = Eject");
            telemetry.addLine("G2 (Operator): D-Pad Up/Down = Depollinator UP/DOWN | Right Bumper = Advance 1 Slot");

            telemetry.update();
        }

        intake.turnOff();
        transfer.stop();
        shooter.stopFlywheels();
        camera.stopDashboardStream();

    }

}
