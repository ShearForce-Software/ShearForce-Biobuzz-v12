package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;
import java.util.Locale;

/**
 * Main Iterative TeleOp OpMode for the BioBuzz robot.
 * Features:
 * - Iterative OpMode structure (extends OpMode) for minimal latency and non-blocking execution.
 * - REV Lynx Module Bulk Caching enabled in AUTO mode.
 * - Dedicated asynchronous background thread for I2C color sensor updates (~50 Hz) preventing main loop stalls.
 * - Rate-limited vision heading drift correction.
 * - Zero object allocations in the hot loop path to prevent garbage collection pauses.
 * - Camera dashboard streaming disabled during match play.
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
@SuppressWarnings("unused")
public class BiobuzzTeleOp extends OpMode {

    // Subsystems
    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ShooterSubsystem shooter;
    private CameraSubsystem camera;

    // Driver States
    private boolean isFieldCentric = false; // Default to Robot Centric mode
    private boolean visionHeadingCorrectionEnabled = true;

    // Operator States
    private boolean intakeOn = false;
    private boolean transferAssumedFull = false;

    // Gamepad state tracking (reused to avoid object allocation in loop)
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    // Timers & background thread
    private Thread colorSensorThread = null;
    private volatile boolean colorSensorThreadRunning = false;
    private final ElapsedTime rateLimitTimer = new ElapsedTime();
    private final ElapsedTime visionCorrectionTimer = new ElapsedTime();

    @Override
    public void init() {
        // Instantiate Subsystems
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);

        // Disable camera streaming during match play to minimize CPU and network load
        camera.stopDashboardStream();

        // Enable bulk caching on all REV Lynx modules (AUTO mode)
        List<LynxModule> allModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Restore exact ending coordinate Pose from Autonomous
        follower.setPose(MatchConfig.currentPose);
        intake.depollinatorUp();
        shooter.hoodMinPosition();
        visionHeadingCorrectionEnabled = false;

        // Start dedicated asynchronous background thread for I2C color sensor updates (~50 Hz)
        colorSensorThreadRunning = true;
        colorSensorThread = new Thread(() -> {
            while (colorSensorThreadRunning && !Thread.currentThread().isInterrupted()) {
                try {
                    if (transfer != null) {
                        transfer.UpdateColorSensors();
                    }
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Ignore transient exceptions
                }
            }
        }, "ColorSensorBackgroundThread");
        colorSensorThread.start();

        rateLimitTimer.reset();
        visionCorrectionTimer.reset();
    }

    @Override
    public void init_loop() {
        if (follower != null) {
            follower.update();
        }
    }

    @Override
    public void start() {
        if (shooter != null) {
            shooter.startFlywheels();
        }
        rateLimitTimer.reset();
    }

    @Override
    public void loop() {
        // Update gamepad states (reusing instances, zero allocations in loop)
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Update subsystem states
        if (follower != null) follower.update();
        if (shooter != null) shooter.update();
        if (transfer != null) transfer.update();

        // =========================================================================
        // GAMEPAD 1: DRIVER CONTROLS (Drive Mode & Heading Reset)
        // =========================================================================

        // Toggle Field Centric / Robot Centric via Gamepad 1 BACK / SELECT button
        if (currentGamepad1.back && !previousGamepad1.back) {
            isFieldCentric = !isFieldCentric;
        }

        // Manually reset heading to 0° via Gamepad 1 START button (for heading drift correction)
        if (currentGamepad1.start && !previousGamepad1.start) {
            Pose currentPose = follower.pose();
            follower.setPose(new Pose(currentPose.x(), currentPose.y(), 0));
        }

        // Joystick inputs (invert y-axis for FTC standard forward = negative)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double targetForward = forward;
        double targetStrafe = strafe;

        // Field Centric transformation using Pinpoint / PedroPathing heading if enabled
        if (isFieldCentric) {
            double robotHeading = follower.pose().heading();
            final double cos = Math.cos(-robotHeading);
            final double sin = Math.sin(-robotHeading);
            targetForward = forward * cos - strafe * sin;
            targetStrafe = forward * sin + strafe * cos;
        }

        // Use PedroPathing methodology for manual drive control
        if (follower != null) {
            follower.manual(targetForward, targetStrafe, turn);
        }

        // Rate-limited vision heading drift correction when stationary (checked every 500ms)
        if (visionHeadingCorrectionEnabled && follower != null && camera != null && visionCorrectionTimer.seconds() >= 0.5) {
            visionCorrectionTimer.reset();
            try {
                double linearSpeed = Math.hypot(follower.localizer.velocity().vx, follower.localizer.velocity().vy);
                double angularSpeed = Math.abs(follower.localizer.velocity().omega);
                if (linearSpeed < 0.05 && angularSpeed < 0.05) {
                    double correctedHeading = camera.getCorrectedPedroHeading();
                    if (!Double.isNaN(correctedHeading)) {
                        follower.setHeading(correctedHeading);
                    }
                }
            } catch (Exception e) {
                // Ignore velocity check exceptions
            }
        }

        // =========================================================================
        // GAMEPAD 2: OPERATOR MECHANISM CONTROLS
        // =========================================================================

        // Shooter Flywheels Toggle (Button Y / Triangle)
        if ((currentGamepad2.triangle || currentGamepad2.y) && !(previousGamepad2.triangle || previousGamepad2.y)) {
            if (gamepad2.share) {
                TransferConfig.AUTO_TRANSFER = !TransferConfig.AUTO_TRANSFER;
            } else {
                if (shooter.isRunning()) {
                    shooter.stopFlywheels();
                } else {
                    shooter.startFlywheels();
                }
            }
        }

        // SHOOT BALLS (Left Bumper)
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            transfer.ShootAllBalls();
            transferAssumedFull = false;
        }

        // Shooter Hood Position uses DPAD LEFT/RIGHT
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            if (!gamepad2.options) {
                shooter.setHoodPosition(shooter.getHoodPosition() - 0.1);
            } else {
                shooter.setPollenPosition();
            }
        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            if (!gamepad2.options) {
                shooter.setHoodPosition(shooter.getHoodPosition() + 0.1);
            } else {
                shooter.setNectarPosition();
            }
        }

        // Intake Subsystem (Cross/A = On, Square/X = Reverse, Circle/B = Off)
        if ((currentGamepad2.cross || currentGamepad2.a) && !(previousGamepad2.cross || previousGamepad2.a)) {
            if (!gamepad2.options) {
                intake.turnOn();
                intakeOn = true;
            }
        } else if ((currentGamepad2.square || currentGamepad2.x) && !(previousGamepad2.square || previousGamepad2.x)) {
            intake.reverse();
            intakeOn = true;
        } else if ((currentGamepad2.circle || currentGamepad2.b) && !(previousGamepad2.circle || previousGamepad2.b)) {
            if (!gamepad2.options) {
                intake.turnOff();
                intakeOn = false;
            }
        }

        // Auto Transfer Logic (non-blocking)
        if (TransferConfig.AUTO_TRANSFER) {
            if (intake.getVelocity() > 0) {
                if (!transfer.IsBusy()) {
                    if (transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_FRONT) || transfer.isRampFrontBallPresent() || transfer.isRampBackBallPresent()) {
                        if (transferAssumedFull) {
                            transfer.StowBalls();
                            intake.reverse();
                        } else {
                            if (transfer.isSlotOccupied(TransferSubsystem.SENSOR_TOP) && transfer.isSlotOccupied(TransferSubsystem.SENSOR_LEFT) && transfer.isSlotOccupied(TransferSubsystem.SENSOR_RIGHT)) {
                                transferAssumedFull = true;
                            }
                            transfer.Load1Ball();
                        }
                    } else if (transferAssumedFull) {
                        transfer.StowBalls();
                        intake.reverse();
                    }
                }
            } else if ((intake.getVelocity() < -intake.targetVelocityTicks * 0.8) &&
                    !(transfer.isSlotOccupied(TransferSubsystem.SENSOR_BOTTOM_FRONT) || transfer.isRampFrontBallPresent() || transfer.isRampBackBallPresent())) {
                intake.turnOff();
                intakeOn = false;
            }
        }

        // Manual Transfer / Load (Right Bumper)
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (!gamepad2.share) {
                transfer.Load1Ball();
            } else {
                transfer.StowBalls();
            }
        }

        // Depollinator Servo UP / DOWN
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            intake.depollinatorUp();
        } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            intake.depollinatorDown();
        }

        // =========================================================================
        // TELEMETRY OUTPUTS (Called at 5 Hz rate -> 200 ms interval)
        // =========================================================================
        if (rateLimitTimer.milliseconds() >= 200) {
            rateLimitTimer.reset();
            PrintTelemetry();
        }
    }

    private void PrintTelemetry() {
        Pose robotPose = follower.pose();

        telemetry.addData("Drive Mode", isFieldCentric ? "FIELD CENTRIC (G1:BACK to switch)" : "ROBOT CENTRIC (DEFAULT) (G1:BACK to switch)");
        telemetry.addData("Heading Reset", "G1:START to reset heading to 0°");
        telemetry.addData("Alliance Color", MatchConfig.allianceColor);
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.1f, %.1f) %.1f°", robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));

        // --- Subsystems Key Data ---
        telemetry.addData("Shooter Flywheels", String.format(Locale.US, "Status: %s | Target: %.0f | Actual: %.1f ticks/s",
                shooter.isRunning() ? "RUNNING" : "STOPPED", shooter.getTargetVelocity(), shooter.getAverageVelocity()));
        telemetry.addData("Intake Motor", String.format(Locale.US, "Status: %s | Actual: %.1f ticks/s",
                intakeOn ? "ON" : "OFF", intake.getVelocity()));
        telemetry.addData("Transfer System", String.format(Locale.US, "Power: %.2f | Raw Ticks: %.0f ",
                transfer.getPower(), transfer.getRawTicks()));
        telemetry.addData("AUTO TRANSFER", TransferConfig.AUTO_TRANSFER ? "ENABLED" : "DISABLED");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop background color sensor thread safely
        colorSensorThreadRunning = false;
        if (colorSensorThread != null) {
            colorSensorThread.interrupt();
        }

        if (intake != null) intake.turnOff();
        if (transfer != null) transfer.stop();
        if (shooter != null) shooter.stopFlywheels();
        if (camera != null) camera.close();
    }
}
