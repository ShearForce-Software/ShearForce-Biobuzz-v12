package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp(name = "Melody Centric Drive Test", group= "BIOBUZZ") // This flags the program as either Autonomous or TeleOp, affecting where it shows up on the Driver Station
    public class MelodyRobotCentricTest extends LinearOpMode { //You may need to add the "extends LinearOpMode" yourself
    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private ShooterSubsystem shooter;
    private CameraSubsystem camera;
    private Command holdCommand = null;
    private boolean isHolding = false;
    //private boolean visionHeadingCorrectionEnabled = true;

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
            //camera.queryCameraControlBoundsAndApply();

            // Restore exact ending coordinate Pose from Autonomous
            follower.setPose(MatchConfig.currentPose);
            intake.depollinatorUp();
            shooter.hoodMinPosition();
            //visionHeadingCorrectionEnabled = false;
        /* This declares the variables we'll use for the motors.  While you technically can name them
        whatever you want here, it's a good idea to match the names in the configuration file */
            DcMotor BackLeft,BackRight,FrontLeft,FrontRight;

            // The next four sections initialize and configure everything needed to use the motors
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

            // This initializes the variables used to convert joystick input into motor velocities or powers
            double ForwardPower,StrafePower,RotatePower,VectorMagnitude;

            // The next two sections initialize everything needed to enable "Slow Mode"
            double FastVelocityFactor = 2000;
            double SlowVelocityFactor = 500;
            double VelocityFactor = FastVelocityFactor;
            boolean SlowModeEnabled = false;

        /* These lines create variables that can save the state of a gamepad - this includes
        what buttons are currently pressed and what position each joystick or trigger is in */
            Gamepad CurrentGamepad1 = new Gamepad();
            Gamepad CurrentGamepad2 = new Gamepad();
            Gamepad PreviousGamepad1 = new Gamepad();
            Gamepad PreviousGamepad2 = new Gamepad();

            ElapsedTime rateLimitTimer = new ElapsedTime();

            waitForStart();
            rateLimitTimer.reset();

            while(opModeIsActive()) {
                follower.update();
                shooter.update();
                transfer.update();

            /* These lines allow you to detect changes with the state of each gamepad.  The "current"
            gamepad state from the previous loops gets copied to the "previous" gamepad state for the
            current loop.  The "current" gamepad state for the current loop is then updated.  By
            comparing the previous gamepad state to the current one, you can do things like distinguish
            between a new button press vs. the button still being held down from a previous press. */
                PreviousGamepad1.copy(CurrentGamepad1);
                PreviousGamepad2.copy(CurrentGamepad2);
                CurrentGamepad1.copy(gamepad1);
                CurrentGamepad2.copy(gamepad2);

            /* The && means "AND" and the ! means "NOT", so this condition is true only if the "A"
            button is currently being pressed but wasn't being pressed in the previous loop */
                if(CurrentGamepad1.a && !PreviousGamepad1.a) {
                    SlowModeEnabled = !SlowModeEnabled;
                    if(SlowModeEnabled)
                        VelocityFactor = SlowVelocityFactor;
                    else
                        VelocityFactor = FastVelocityFactor;
                }

                ForwardPower = -gamepad1.left_stick_y;
                StrafePower = gamepad1.left_stick_x;
                RotatePower = gamepad1.right_stick_x;

            /* The "Math" library provides a series of math functions like finding the max from a
            series of numbers or determining the absolute value of a quantity */
                VectorMagnitude = Math.max(Math.abs(ForwardPower)+Math.abs(StrafePower)+Math.abs(RotatePower),1);

                ((DcMotorEx) BackLeft).setVelocity(VelocityFactor/VectorMagnitude*(ForwardPower-StrafePower+RotatePower));
                ((DcMotorEx) BackRight).setVelocity(VelocityFactor/VectorMagnitude*(ForwardPower+StrafePower-RotatePower));
                ((DcMotorEx) FrontLeft).setVelocity(VelocityFactor/VectorMagnitude*(ForwardPower+StrafePower+RotatePower));
                ((DcMotorEx) FrontRight).setVelocity(VelocityFactor/VectorMagnitude*(ForwardPower-StrafePower-RotatePower));

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
                        intakeOn=true;
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
                        intakeOn = false;
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

                Pose goalPose = MatchConfig.getAudienceGoalPose();
                Pose robotPose = follower.pose();
                double deltaX = goalPose.x() - robotPose.x();
                double deltaY = goalPose.y() - robotPose.y();
                double odomDistance = Math.hypot(deltaX, deltaY);
                double odomBearingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(deltaY, deltaX) - robotPose.heading()));
                // The next three sections set and update all telemetry
                telemetry.addLine("=== Match & Target Info ===");
                telemetry.addData("Alliance Color", MatchConfig.allianceColor);
                telemetry.addData("Hive State", MatchConfig.isAudienceGoalUp() ? "AUDIENCE GOAL UP" : "AUDIENCE GOAL DOWN");
                //telemetry.addData("Vision Heading Correction", visionHeadingCorrectionEnabled ? "ENABLED (G1:Circle to toggle)" : "DISABLED");
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
                }

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

