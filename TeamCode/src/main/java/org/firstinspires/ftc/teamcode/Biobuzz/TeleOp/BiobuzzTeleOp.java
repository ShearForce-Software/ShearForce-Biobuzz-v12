package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferConfig;
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

    private boolean transferAssumedFull = false;



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

        // Pre-spool flywheels for TeleOp readiness as requested
        shooter.startFlywheels();

        while (opModeIsActive()) {
            // Update subsystem states
            follower.update();
            shooter.update();
            transfer.update();

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

            // Shooter Flywheels Toggle (Button Y / Triangle)
            if (gamepad2.triangleWasPressed()) {
                // share + triangle = toggle auto transfer system on/off
                if (gamepad2.share) {
                    if (TransferConfig.AUTO_TRANSFER) {
                        TransferConfig.AUTO_TRANSFER = false;
                    }
                    else {
                        TransferConfig.AUTO_TRANSFER = true;
                    }
                }
                // triangle by itself means start/stop shooter flywheels
                else {
                    if (shooter.isRunning()) {
                        shooter.stopFlywheels();
                    } else {
                        shooter.startFlywheels();
                    }
                }
            }

            // SHOOT BALLS
            if (gamepad2.leftBumperWasPressed()) {
                //transfer.Shoot1Ball();
                transfer.ShootAllBalls();
                transferAssumedFull = false;
            }

            // Shooter Hood Position uses DPAD LEFT/RIGHT
            // Shooter Pollen/Nectar Servo Position uses DPAD LEFT/RIGHT + Options
            if (gamepad2.dpadLeftWasPressed()) {
                if (!gamepad2.options) {
                    shooter.setHoodPosition(shooter.getHoodPosition() - 0.1);
                }
                else {
                    shooter.setPollenPosition();
                }
            } else if (gamepad2.dpadRightWasPressed()) {
                if (!gamepad2.options) {
                    shooter.setHoodPosition(shooter.getHoodPosition() + 0.1);
                }
                else {
                    shooter.setNectarPosition();
                }
            }

            // Intake Subsystem
            if (gamepad2.crossWasPressed()) {
                if (!gamepad2.options) {
                    intake.turnOn();
                }
            } else if (gamepad2.squareWasPressed()) {
                intake.reverse();
            } else if (gamepad2.circleWasPressed()) {
                if (!gamepad2.options) {
                    intake.turnOff();
                }
            }

            // if Auto Transfer Logic enabled (toggle on/off by using SHARE + TRIANGLE)
            if (TransferConfig.AUTO_TRANSFER) {
                // if intake is turned on (bringing in new balls)
                if (intake.getVelocity() > 0) {
                    // if transfer is not already in progress
                    if (!transfer.IsBusy()) {
                        // if a ball is about to be in the bottom slot
                        if (transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_FRONT) || intake.isRampFrontBallPresent() || intake.isRampBackBallPresent()) {
                            // if intake is already full then we are controlling too many balls!!!
                            if (transferAssumedFull) {
                                transfer.StowBalls();
                                // spit out any extra balls
                                intake.reverse();
                            } else {
                                // if Top, Left, and Right slots already reporting full
                                if (transfer.isSlotOccupied(TransferSubsystem.SENSOR_TOP) && transfer.isSlotOccupied(TransferSubsystem.SENSOR_LEFT) && transfer.isSlotOccupied(TransferSubsystem.SENSOR_RIGHT)) {
                                    transferAssumedFull = true;
                                }
                                transfer.Load1Ball();
                            }
                        } else if (transferAssumedFull) {
                            transfer.StowBalls();
                            // spit out anything else not being detected yet
                            intake.reverse();
                        }
                    }
                }
                // else if out-taking and velocity has reached at least 80% of target velocity and no more extra balls are being detected
                else if ((intake.getVelocity() < -intake.targetVelocityTicks * 0.8) &&
                        !(transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_FRONT) || intake.isRampFrontBallPresent() || intake.isRampBackBallPresent()) ) {
                    // assume it is safe to turn off the intake subsystem
                    intake.turnOff();
                }
            }

            // Manual Transfer / Load
            if (gamepad2.rightBumperWasPressed()) {
                if (!gamepad2.share) {
                    transfer.Load1Ball();
                }
                else {
                    transfer.StowBalls();
                }
            }

            // Depollinator Servo UP / DOWN
            if (gamepad2.dpadUpWasPressed()) {
                intake.depollinatorUp();
            } else if (gamepad2.dpadDownWasPressed()) {
                intake.depollinatorDown();
            }



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
            telemetry.addData("Shooter Servos", String.format(Locale.US, "Mode: %s | Hood: %.0f ",
                    shooter.isPollenPosition() ? "POLLEN" : "NECTAR", shooter.getHoodPosition()));
            telemetry.addData("Intake Motor", String.format(Locale.US, "Status: %s | Target: %.0f | Actual: %.1f ticks/s",
                    intakeOn ? "ON" : "OFF", intake.targetVelocityTicks, intake.getVelocity()));
            telemetry.addData("Transfer System", String.format(Locale.US, "Power: %.2f | Raw Ticks: %.0f ",
                    transfer.getPower(), transfer.getRawTicks()));
            telemetry.addData("AUTO TRANSFER", String.format(Locale.US, "Mode: %s ",
                    TransferConfig.AUTO_TRANSFER ? "ENABLED" : "DISABLED"));

            // --- Controlled Ball Totals (Pollen vs Nectar) & Sensor Readings ---
            int totalBalls = 0;
            int totalPollen = 0;
            int totalNectar = 0;

            TransferSubsystem.BallColor[] slotColors = transfer.getSlotColors();
            for (TransferSubsystem.BallColor color : slotColors) {
                if (color == TransferSubsystem.BallColor.YELLOW) {
                    ++totalPollen;
                    ++totalBalls;
                } else if (color == TransferSubsystem.BallColor.RED || color == TransferSubsystem.BallColor.BLUE) {
                    ++totalNectar;
                    ++totalBalls;
                } else if (color != TransferSubsystem.BallColor.NONE && color != TransferSubsystem.BallColor.UNKNOWN) {
                    ++totalBalls;
                }
            }

            // Calculate distances
            double topDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_TOP);
            String topDistStr = (topDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", topDist);
            double leftDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_LEFT);
            String leftDistStr = (leftDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", leftDist);
            double rightDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_RIGHT);
            String rightDistStr = (rightDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rightDist);
            double bbDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_BACK);
            String bbDistStr = (bbDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", bbDist);
            double bfDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            String bfDistStr = (bfDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", bfDist);
            double rampBackDist = intake.getRampBackDistanceMm();
            String rampBackDistStr = (rampBackDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rampBackDist);
            double rampFrontDist = intake.getRampFrontDistanceMm();
            String rampFrontDistStr = (rampFrontDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rampFrontDist);


            telemetry.addLine("=== Color & Distance Sensors ===");
            telemetry.addData("Total", String.format(Locale.US, "Balls: %d/4 | Pollen: %d | Nectar: %d",
                    totalBalls, totalPollen, totalNectar));

            telemetry.addData("Ramp Back   -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    intake.isRampBackBallPresent() ? "YES" : "NO ", rampBackDistStr, intake.detectRampBackColor()));
            telemetry.addData("Ramp Front  -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    intake.isRampFrontBallPresent() ? "YES" : "NO ", rampFrontDistStr, intake.detectRampFrontColor()));
            telemetry.addData("Bottom Back -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_BACK) ? "YES" : "NO ", bbDistStr, transfer.detectSlotColor(TransferSubsystem.SENSOR_BOTTOM_BACK)));
            telemetry.addData("Bottom Front-", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_FRONT) ? "YES" : "NO ", bfDistStr, transfer.detectSlotColor(TransferSubsystem.SENSOR_BOTTOM_FRONT)));

            telemetry.addData("Top Slot    -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_TOP) ? "YES" : "NO ", topDistStr, transfer.detectSlotColor(TransferSubsystem.SENSOR_TOP)));
            telemetry.addData("Left Slot   -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_LEFT) ? "YES" : "NO ", leftDistStr, transfer.detectSlotColor(TransferSubsystem.SENSOR_LEFT)));
            telemetry.addData("Right Slot  -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                    transfer.isSlotOccupied(TransferSubsystem.SENSOR_RIGHT) ? "YES" : "NO ", rightDistStr, transfer.detectSlotColor(TransferSubsystem.SENSOR_RIGHT)));

            // --- Controls Summary ---
            telemetry.addLine("=== Controls Summary ===");
            telemetry.addLine("G1: Joysticks = Field   | X-Cross = Hold Mode    | Circle = Vision Correct");
            telemetry.addLine("G2: Left Bumper = Shoot | DPAD Left/Right = Hood | Triangle = Flywheels");
            telemetry.addLine("G2: X-Cross = Intake ON | Circle = Intake OFF    | Square = EJECT");
            telemetry.addLine("G2: D-Pad Up/Down = Depollinator UP/DOWN         | Right Bumper = 1 Slot Load");
            telemetry.addLine("G2: Right Bumper + SHARE = 1/2 Slot Load (STOW)");
            telemetry.addLine("G2: Triangle + SHARE = Auto Transfer MODE Toggle");
            telemetry.addLine("G2: D-Pad Left/Right + OPTIONS = Pollen/Nectar Shoot mode");

            telemetry.update();
        }

        intake.turnOff();
        transfer.stop();
        shooter.stopFlywheels();
        camera.stopDashboardStream();

    }

}
