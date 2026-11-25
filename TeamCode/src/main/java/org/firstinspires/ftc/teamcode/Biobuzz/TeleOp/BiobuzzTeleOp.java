package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

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

    IMU imu;

    // Driver States
    private boolean isFieldCentric = false; // Default to Robot Centric mode
    private boolean visionHeadingCorrectionEnabled = false;

    // Operator States
    private boolean intakeOn = false;
    private boolean transferAssumedFull = false;

    // Gamepad state tracking (reused to avoid object allocation in loop)
    //private final Gamepad currentGamepad1 = new Gamepad();
    //private final Gamepad previousGamepad1 = new Gamepad();
    //private final Gamepad currentGamepad2 = new Gamepad();
    //private final Gamepad previousGamepad2 = new Gamepad();

    // Timers
    private final ElapsedTime telemetryUpdateTimer = new ElapsedTime();
    private final ElapsedTime visionCorrectionTimer = new ElapsedTime();
    private final ElapsedTime hiveUpdateTimer = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void init() {
        // Instantiate Subsystems
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap, intake);
        shooter = new ShooterSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront_centerOdometry");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear_rightOdometry");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Disable camera streaming during match play to minimize CPU and network load
        camera.stopDashboardStream();

        // Enable bulk caching on all REV Lynx modules (AUTO mode)
        List<LynxModule> allModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Restore exact ending coordinate Pose from Autonomous (with sanity check fallback)
        if (MatchConfig.isValidPose(MatchConfig.currentPose)) {
            follower.setPose(MatchConfig.currentPose);
        } else {
            // default position if current pose is invalid, user can use reset function to fix the heading
            follower.setPose(new Pose(99, 99, Math.toRadians(99)));
        }
        intake.depollinatorUp();
        shooter.hoodMinPosition();
        visionHeadingCorrectionEnabled = false;

        // Start dedicated background thread for I2C color sensor updates (~15 Hz)
        if (transfer != null) {
            transfer.startColorSensorThread();
        }

        telemetryUpdateTimer.reset();
        visionCorrectionTimer.reset();
        hiveUpdateTimer.reset();
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
        telemetryUpdateTimer.reset();
        hiveUpdateTimer.reset();
    }

    @Override
    public void loop() {
        // Update gamepad states (reusing instances, zero allocations in loop)
        //previousGamepad1.copy(currentGamepad1);
        //previousGamepad2.copy(currentGamepad2);
        //currentGamepad1.copy(gamepad1);
        //currentGamepad2.copy(gamepad2);

        // Update subsystem states
        if (follower != null) follower.update();
        if (shooter != null) shooter.update();
        if (transfer != null) transfer.update();

        // =========================================================================
        // GAMEPAD 1: DRIVER CONTROLS (Drive Mode & Heading Reset)
        // =========================================================================

        if (gamepad1.triangleWasPressed())
        {
            // G1: Triangle + Options = Toggle Field Centric / Robot Centric via Gamepad 1
            if (gamepad1.options) {
                isFieldCentric = !isFieldCentric;
            } else {
                // G1: Triangle = Manually reset heading to 0° via Gamepad 1 (for heading drift correction)
                Pose currentPose = follower.pose();
                follower.setPose(new Pose(currentPose.x(), currentPose.y(), 0));
                imu.resetYaw();
            }
        }

        // Joystick inputs (invert y-axis for FTC standard forward = negative)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Driver Assistance: Cardinal Heading Snap on Gamepad 1 DPAD buttons (must hold buttons down until gets to the desired heading)
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            double targetHeading = 0;
            if (gamepad1.dpad_up) targetHeading = Math.toRadians(90);        // 90° (Facing TV)
            else if (gamepad1.dpad_down) targetHeading = Math.toRadians(270); // 270° (Facing Audience)
            else if (gamepad1.dpad_left) targetHeading = Math.toRadians(180); // 180° (Facing Red Drivers)
            else if (gamepad1.dpad_right) targetHeading = Math.toRadians(0);  // 0° (Facing Blue Drivers)

            double currentHeading = follower.pose().heading();
            double headingError = AngleUnit.normalizeRadians(targetHeading - currentHeading);

            // Deadband tolerance window: if within 1.0 degree, stop turning to prevent hunting/bouncing
            if (Math.abs(headingError) < Math.toRadians(1.0)) {
                turn = 0;
            } else {
                double angularVelocity = follower.localizer.velocity().omega; // rad/sec
                double kP_headingSnap = 0.9;
                double kD_headingSnap = 0.08;
                turn = Range.clip((headingError * kP_headingSnap) - (angularVelocity * kD_headingSnap), -0.65, 0.65);
            }
        } else if (gamepad1.square) {
            // Driver Assistance: Auto-aim Shooter (mounted 90° right of robot forward) at active Hive Goal when Square is held
            Pose robotPose = follower.pose();
            Pose goalPose = MatchConfig.getActiveGoalPose();
            double deltaX = goalPose.x() - robotPose.x();
            double deltaY = goalPose.y() - robotPose.y();

            // Target field angle from robot to active goal
            double targetFieldAngle = Math.atan2(deltaY, deltaX);

            // Shooter is offset 90° right (-90° / -PI/2) from robot forward.
            // Therefore, robot forward heading must equal (targetFieldAngle + 90°) so shooter points at goal.
            double targetRobotHeading = targetFieldAngle + (Math.PI / 2.0);

            double currentHeading = robotPose.heading();
            double headingError = AngleUnit.normalizeRadians(targetRobotHeading - currentHeading);

            if (Math.abs(headingError) < Math.toRadians(1.0)) {
                turn = 0;
            } else {
                double angularVelocity = follower.localizer.velocity().omega; // rad/sec
                double kP_aim = 1.0;
                double kD_aim = 0.08;
                turn = Range.clip((headingError * kP_aim) - (angularVelocity * kD_aim), -0.70, 0.70);
            }
        }

        // Command the robot to drive with the joystick / snap inputs
        DriveRobot(forward, strafe, turn);

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
        if (gamepad2.triangleWasPressed()) {
            // share + triangle = toggle auto transfer system on/off
            if (gamepad2.share) {
                TransferConfig.AUTO_TRANSFER = !TransferConfig.AUTO_TRANSFER;
            } else {
                // triangle by itself means start/stop shooter flywheels
                if (shooter.isRunning()) {
                    shooter.stopFlywheels();
                } else {
                    shooter.startFlywheels();
                }
            }
        }

        // SHOOT BALLS (Left Bumper = Shoot All, Left Trigger = Shoot 1 Ball)
        if (gamepad2.leftBumperWasPressed()) {
            transfer.ShootAllBalls();
            transferAssumedFull = false;
        } else if (gamepad2.leftTriggerWasPressed()) {
            transfer.Shoot1Ball();
            transferAssumedFull = false;
        }

        // TURN OFF TRANSFER (Right Trigger)
        if (gamepad2.rightTriggerWasPressed()) {
            transfer.stop();
            transferAssumedFull = true;
        }

        // Shooter Hood Position uses DPAD LEFT/RIGHT
        // Shooter Pollen/Nectar Servo Position uses DPAD LEFT/RIGHT + Options
        if (gamepad2.dpadLeftWasPressed()) {
            if (!gamepad2.options) {
                shooter.setHoodPosition(shooter.getHoodPosition() - 0.1);
            } else {
                shooter.setPollenPosition();
            }
        } else if (gamepad2.dpadRightWasPressed()) {
            if (!gamepad2.options) {
                shooter.setHoodPosition(shooter.getHoodPosition() + 0.1);
            } else {
                shooter.setNectarPosition();
            }
        }

        // Intake Subsystem (Cross/A = On, Square/X = Reverse, Circle/B = Off)
        if (gamepad2.crossWasPressed()) {
            if (!gamepad2.options) {
                intake.turnOn();
                intakeOn = true;
            }
        } else if (gamepad2.squareWasPressed()) {
            if (gamepad2.share) {
                // G2: Square + SHARE = Reset Transfer Thru-Bore Encoder Position Count to 0
                if (transfer != null) {
                    transfer.resetEncoder();
                }
            } else {
                intake.reverse();
                intakeOn = true;
            }
        } else if (gamepad2.circleWasPressed()) {
            if (!gamepad2.options) {
                intake.turnOff();
                intakeOn = false;
            }
        }

        // Manual Transfer / Load (Right Bumper)
        if (gamepad2.rightBumperWasPressed()) {
            if (!gamepad2.share) {
                transfer.Load1Ball();
            } else {
                transfer.StowBalls();
            }
        }

        // Depollinator Servo UP / DOWN = DPAD UP / DOWN
        // Shooter Flywheels SPEED = OPTIONS + DPAD UP/DOWN
        if (gamepad2.dpadUpWasPressed()) {
            if (gamepad2.options) {
                shooter.setVelocity(shooter.getTargetVelocity() + 50.0);
            } else {
                intake.depollinatorUp();
            }
        } else if (gamepad2.dpadDownWasPressed()) {
            if (gamepad2.options) {
                shooter.setVelocity(shooter.getTargetVelocity() - 50.0);
            } else {
                intake.depollinatorDown();
            }
        }

        // Rate-limited Active Hive Position Update (5 Hz rate -> 200 ms interval)
        if (hiveUpdateTimer.milliseconds() >= 200) {
            hiveUpdateTimer.reset();
            if (camera != null && follower != null) {
                camera.updateActiveHiveFromPosition(follower.pose());
            }
        }

        // =========================================================================
        // TELEMETRY OUTPUTS (Called at 5 Hz rate -> 200 ms interval)
        // =========================================================================
        if (telemetryUpdateTimer.milliseconds() >= 200) {
            telemetryUpdateTimer.reset();
            PrintTelemetry();
        }
    }

    private void DriveRobot(double forward, double strafe, double turn) {
        double targetForward = forward;
        double targetStrafe = strafe;

        // Field Centric transformation using Pinpoint / PedroPathing heading if enabled
        if (isFieldCentric) {
            // Driver station orientation offset: 0 for Red, 180° (PI) for Blue
            double allianceHeadingOffset = MatchConfig.isBlueAlliance() ? Math.PI : 0.0;

            // Effective heading relative to the driver standing behind driver station glass
            double robotHeading = follower.pose().heading() - allianceHeadingOffset;

            final double cos = Math.cos(-robotHeading);
            final double sin = Math.sin(-robotHeading);
            targetForward = forward * cos - strafe * sin;
            targetStrafe = forward * sin + strafe * cos;
        }

        // Apply 10% boost to strafe commands to compensate for mecanum wheel friction
        targetStrafe *= 1.1;

        double frontLeftPower  = targetForward + targetStrafe + turn;
        double frontRightPower = targetForward - targetStrafe - turn;
        double backLeftPower   = targetForward - targetStrafe + turn;
        double backRightPower  = targetForward + targetStrafe - turn;

        double max;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void PrintTelemetry() {
        // --- Match & Target Info ---
        Pose robotPose = follower.pose();
		
        // Calculate distance to active goal using pedro pathing data and MatchConfig
        Pose goalPose = MatchConfig.getActiveGoalPose();	
        double deltaX = goalPose.x() - robotPose.x();
        double deltaY = goalPose.y() - robotPose.y();
        double odomDistance = Math.hypot(deltaX, deltaY);
        double odomBearingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(deltaY, deltaX) - robotPose.heading()));

        // Report AprilTag calculated distance to goal
        //AprilTagClusterDetection closestCluster = camera.getClosestClusterDetection();
        //double cameraDistance = (closestCluster != null && closestCluster.ftcPose != null) ? closestCluster.ftcPose.range : Double.NaN;
        //double cameraBearing = (closestCluster != null && closestCluster.ftcPose != null) ? closestCluster.ftcPose.bearing : Double.NaN;
		
        telemetry.addData("Drive Mode", isFieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");
        telemetry.addLine("=== Match & Target Info ===");
        telemetry.addData("Alliance Color", MatchConfig.allianceColor);
        telemetry.addData("Hive State", MatchConfig.isAudienceGoalUp() ? "AUDIENCE GOAL UP" : "AUDIENCE GOAL DOWN");
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.1f, %.1f) %.1f°", robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
        telemetry.addData("Distance to Goal", String.format(Locale.US, "Odom: %.1f in ", odomDistance));
        telemetry.addData("Bearing to Goal", String.format(Locale.US, "Odom: %.1f° ", odomBearingDeg));

        // --- Subsystems Key Data ---
        telemetry.addLine("=== Subsystems Key Data ===");
        telemetry.addData("Shooter Flywheels", String.format(Locale.US, "Status: %s | Target: %.0f | Actual: %.1f ticks/s",
                shooter.isRunning() ? "RUNNING" : "STOPPED", shooter.getTargetVelocity(), shooter.getAverageVelocity()));
        telemetry.addData("Shooter Servos", String.format(Locale.US, "Mode: %s | Hood: %.0f ",
            shooter.isPollenPosition() ? "POLLEN" : "NECTAR", shooter.getHoodPosition()));
        telemetry.addData("Intake Motor", String.format(Locale.US, "Status: %s | Actual: %.1f ticks/s",
                intakeOn ? "ON" : "OFF", intake.getVelocity()));
        telemetry.addData("Transfer System", String.format(Locale.US, "Power: %.2f | Raw Ticks: %.0f ",
                transfer.getPower(), transfer.getRawTicks()));
        telemetry.addData("AUTO TRANSFER", TransferConfig.AUTO_TRANSFER ? "ENABLED" : "DISABLED");

        // --- Color & Distance Sensors ---
        telemetry.addLine("=== Color & Distance Sensors ===");
        if (transfer != null) {
            int transferBalls = transfer.getBallCount();
            int rampBalls = transfer.getRampBallCount();
            int totalBalls = transferBalls + rampBalls;
            telemetry.addData("Total Balls Detected", String.format(Locale.US, "%d (Transfer: %d | Ramp: %d)", totalBalls, transferBalls, rampBalls));

            double distTop = transfer.getDistanceMm(TransferSubsystem.SENSOR_TOP);
            double distLeft = transfer.getDistanceMm(TransferSubsystem.SENSOR_LEFT);
            double distRight = transfer.getDistanceMm(TransferSubsystem.SENSOR_RIGHT);
            double distBotFront = transfer.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            double distRampBack = transfer.getDistanceMm(TransferSubsystem.SENSOR_RAMP_BACK);
            double distRampFront = transfer.getDistanceMm(TransferSubsystem.SENSOR_RAMP_FRONT);

            telemetry.addData("Top Sensor", String.format(Locale.US, "Color: %s | Dist: %s",
                    transfer.detectSensorColor(TransferSubsystem.SENSOR_TOP),
                    distTop == Double.MAX_VALUE ? "N/A" : String.format(Locale.US, "%.1f mm", distTop)));
            telemetry.addData("Left Sensor", String.format(Locale.US, "Color: %s | Dist: %s",
                    transfer.detectSensorColor(TransferSubsystem.SENSOR_LEFT),
                    distLeft == Double.MAX_VALUE ? "N/A" : String.format(Locale.US, "%.1f mm", distLeft)));
            telemetry.addData("Right Sensor", String.format(Locale.US, "Color: %s | Dist: %s",
                    transfer.detectSensorColor(TransferSubsystem.SENSOR_RIGHT),
                    distRight == Double.MAX_VALUE ? "N/A" : String.format(Locale.US, "%.1f mm", distRight)));
            telemetry.addData("Bottom Sensor", String.format(Locale.US, "Color: %s | Dist: %s",
                    transfer.detectSensorColor(TransferSubsystem.SENSOR_BOTTOM_FRONT),
                    distBotFront == Double.MAX_VALUE ? "N/A" : String.format(Locale.US, "%.1f mm", distBotFront)));
            telemetry.addData("Ramp Back Sensor", String.format(Locale.US, "Color: %s | Dist: %s",
                    transfer.detectSensorColor(TransferSubsystem.SENSOR_RAMP_BACK),
                    distRampBack == Double.MAX_VALUE ? "N/A" : String.format(Locale.US, "%.1f mm", distRampBack)));
            telemetry.addData("Ramp Front Sensor", String.format(Locale.US, "Color: %s | Dist: %s",
                    transfer.detectSensorColor(TransferSubsystem.SENSOR_RAMP_FRONT),
                    distRampFront == Double.MAX_VALUE ? "N/A" : String.format(Locale.US, "%.1f mm", distRampFront)));
        }

        // --- Controls Summary ---
        telemetry.addLine("=== Controls Summary ===");
        telemetry.addLine("G1: Joysticks = Drive   | D-Pad = Snap Heading   | Square = Aim Shooter at Goal");
        telemetry.addLine("G1: Triangle = Reset Heading | Triangle + Options = Toggle Field/Robot Mode");
        telemetry.addLine("G1: Circle = Toggle Auto Vision Heading Correction Mode");

        telemetry.addLine("G2: Left Bumper = Shoot All | Left Trigger = Shoot 1 | Right Trigger = Stop Transfer");
        telemetry.addLine("G2: DPAD Left/Right = Hood | Triangle = Flywheels | Square = EJECT");
        telemetry.addLine("G2: Square + SHARE = Reset Transfer Encoder");
        telemetry.addLine("G2: D-Pad Up/Down = Depollinator UP/DOWN         | Right Bumper = 1 Slot Load");
        telemetry.addLine("G2: Right Bumper + SHARE = 1/2 Slot Load (STOW)");
        telemetry.addLine("G2: Triangle + SHARE = Auto Transfer MODE Toggle");
        telemetry.addLine("G2: D-Pad Left/Right + OPTIONS = Pollen/Nectar Shoot mode");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop background color sensor thread safely
        if (transfer != null) {
            transfer.stopColorSensorThread();
            transfer.stop();
        }

        if (intake != null) intake.turnOff();
        if (shooter != null) shooter.stopFlywheels();
        if (camera != null) camera.close();
    }
}
