package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.Locale;

@TeleOp(name = "Melody Centric Drive Test", group= "BIOBUZZ")
@SuppressWarnings("unused")
public class MelodyRobotCentricTest extends LinearOpMode {
    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ShooterSubsystem shooter;
    private CameraSubsystem camera;
    private Command holdCommand = null;
    private boolean isHolding = false;

    // Operator States
    private boolean intakeOn = false;
    private boolean transferAssumedFull = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        transfer = new TransferSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);

        // Restore exact ending coordinate Pose from Autonomous
        follower.setPose(MatchConfig.currentPose);
        intake.depollinatorUp();
        shooter.hoodMinPosition();

        DcMotor BackLeft, BackRight, FrontLeft, FrontRight;

        BackLeft = hardwareMap.get(DcMotor.class, "leftRear");
        BackRight = hardwareMap.get(DcMotor.class, "rightRear_rightOdometry");
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront_centerOdometry");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double ForwardPower, StrafePower, RotatePower, VectorMagnitude;

        double FastVelocityFactor = 2000;
        double SlowVelocityFactor = 500;
        double VelocityFactor = FastVelocityFactor;
        boolean SlowModeEnabled = false;

        Gamepad CurrentGamepad1 = new Gamepad();
        Gamepad CurrentGamepad2 = new Gamepad();
        Gamepad PreviousGamepad1 = new Gamepad();
        Gamepad PreviousGamepad2 = new Gamepad();

        ElapsedTime rateLimitTimer = new ElapsedTime();
        ElapsedTime colorUpdateTimer = new ElapsedTime();

        waitForStart();
        rateLimitTimer.reset();
        colorUpdateTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            shooter.update();
            transfer.update();

            // Call UpdateColorSensors at 10 Hz rate (every 100 ms)
            if (colorUpdateTimer.milliseconds() >= 100) {
                colorUpdateTimer.reset();
                transfer.UpdateColorSensors();
            }

            PreviousGamepad1.copy(CurrentGamepad1);
            PreviousGamepad2.copy(CurrentGamepad2);
            CurrentGamepad1.copy(gamepad1);
            CurrentGamepad2.copy(gamepad2);

            if (CurrentGamepad1.a && !PreviousGamepad1.a) {
                SlowModeEnabled = !SlowModeEnabled;
                if (SlowModeEnabled)
                    VelocityFactor = SlowVelocityFactor;
                else
                    VelocityFactor = FastVelocityFactor;
            }

            ForwardPower = -gamepad1.left_stick_y;
            StrafePower = gamepad1.left_stick_x;
            RotatePower = gamepad1.right_stick_x;

            VectorMagnitude = Math.max(Math.abs(ForwardPower) + Math.abs(StrafePower) + Math.abs(RotatePower), 1);

            ((DcMotorEx) BackLeft).setVelocity(VelocityFactor / VectorMagnitude * (ForwardPower - StrafePower + RotatePower));
            ((DcMotorEx) BackRight).setVelocity(VelocityFactor / VectorMagnitude * (ForwardPower + StrafePower - RotatePower));
            ((DcMotorEx) FrontLeft).setVelocity(VelocityFactor / VectorMagnitude * (ForwardPower + StrafePower + RotatePower));
            ((DcMotorEx) FrontRight).setVelocity(VelocityFactor / VectorMagnitude * (ForwardPower - StrafePower - RotatePower));

            if (gamepad2.triangleWasPressed()) {
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

            // SHOOT BALLS
            if (gamepad2.leftBumperWasPressed()) {
                transfer.ShootAllBalls();
                transferAssumedFull = false;
            }

            // Shooter Hood Position uses DPAD LEFT/RIGHT
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

            // Intake Subsystem
            if (gamepad2.crossWasPressed()) {
                if (!gamepad2.options) {
                    intake.turnOn();
                    intakeOn = true;
                }
            } else if (gamepad2.squareWasPressed()) {
                intake.reverse();
                intakeOn = true;
            } else if (gamepad2.circleWasPressed()) {
                if (!gamepad2.options) {
                    intake.turnOff();
                    intakeOn = false;
                }
            }

            // if Auto Transfer Logic enabled
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

            // Manual Transfer / Load
            if (gamepad2.rightBumperWasPressed()) {
                if (!gamepad2.share) {
                    transfer.Load1Ball();
                } else {
                    transfer.StowBalls();
                }
            }

            // Depollinator Servo UP / DOWN
            if (gamepad2.dpadUpWasPressed()) {
                intake.depollinatorUp();
            } else if (gamepad2.dpadDownWasPressed()) {
                intake.depollinatorDown();
            }

            Pose goalPose = MatchConfig.getAudienceGoalPose();
            Pose robotPose = follower.pose();
            double deltaX = goalPose.x() - robotPose.x();
            double deltaY = goalPose.y() - robotPose.y();
            double odomDistance = Math.hypot(deltaX, deltaY);
            double odomBearingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(deltaY, deltaX) - robotPose.heading()));

            telemetry.addLine("=== Match & Target Info ===");
            telemetry.addData("Alliance Color", MatchConfig.allianceColor);
            telemetry.addData("Hive State", MatchConfig.isAudienceGoalUp() ? "AUDIENCE GOAL UP" : "AUDIENCE GOAL DOWN");
            telemetry.addData("Robot Pose", String.format(Locale.US, "(%.1f, %.1f) %.1f°", robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));

            telemetry.addData("Forward Power", ForwardPower);
            telemetry.addData("Strafe Power", StrafePower);
            telemetry.addData("Rotate Power", RotatePower);
            telemetry.addData("Vector Magnitude", VectorMagnitude);

            telemetry.addData("Back Left Velocity", ((DcMotorEx) BackLeft).getVelocity());
            telemetry.addData("Back Right Velocity", ((DcMotorEx) BackRight).getVelocity());
            telemetry.addData("Front Left Velocity", ((DcMotorEx) FrontLeft).getVelocity());
            telemetry.addData("Front Right Velocity", ((DcMotorEx) FrontRight).getVelocity());

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

            if (rateLimitTimer.milliseconds() >= 100) {
                rateLimitTimer.reset();

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
                double rampBackDist = transfer.getRampBackDistanceMm();
                String rampBackDistStr = (rampBackDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rampBackDist);
                double rampFrontDist = transfer.getRampFrontDistanceMm();
                String rampFrontDistStr = (rampFrontDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rampFrontDist);

                telemetry.addLine("=== Color & Distance Sensors ===");
                telemetry.addData("Total", String.format(Locale.US, "Balls: %d/4 | Pollen: %d | Nectar: %d",
                        totalBalls, totalPollen, totalNectar));

                telemetry.addData("Ramp Back   -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                        transfer.isRampBackBallPresent() ? "YES" : "NO ", rampBackDistStr, transfer.detectRampBackColor()));
                telemetry.addData("Ramp Front  -", String.format(Locale.US, "%s | Dist: %s | Color: %s",
                        transfer.isRampFrontBallPresent() ? "YES" : "NO ", rampFrontDistStr, transfer.detectRampFrontColor()));
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
            }

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
