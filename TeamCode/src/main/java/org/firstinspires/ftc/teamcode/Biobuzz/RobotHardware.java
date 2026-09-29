package org.firstinspires.ftc.teamcode.Biobuzz;

import static org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive.PARAMS;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import org.firstinspires.ftc.teamcode.Gericka.GerickaDatalog;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

//import org.firstinspires.ftc.teamcode.PinpointLocalizer;
//import org.firstinspires.ftc.teamcode.testign123;

public class RobotHardware {
    public static final String ALLIANCE_KEY = "Alliance";
    public static final String FINAL_X_POSITION = "Final_X_Position";
    public static final String FINAL_Y_POSITION = "Final_Y_Position";
    public static final String FINAL_HEADING_DEGREES = "Final_Heading_Degrees";

    public int detectedTagId;
    IMU imu;
    public double imuOffsetInDegrees = 0.0;
    double imuPosition = 0;

    LinearOpMode opMode;
    public static double autoTimeLeft = 0.0;
    boolean IsDriverControl;
    boolean IsFieldCentric;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private DcMotorEx intakeMotor;
    public DcMotorEx shooterMotorRight;
    public DcMotorEx shooterMotorLeft;
    private DcMotorEx turretMotor;
    private Servo launchRampServo;

    //Webcam
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public static double INTAKE_POWER = 0.90;
    public static double SHOOT_POWER = 1.0;
    private boolean autoTurretMode = true;
    private boolean autoIntakeMode = false;
    private boolean autoShooterMode = false;
    private boolean autoTurretResetMode = true;
    private boolean autoHoodMode = true;
    private double shooterTargetRPM = 0.0;
    public final double yawScalar = 0.99877591688;

    public static boolean shooterPIDF_Enabled = false;

    public static boolean turretPIDF_Enabled = true;

    public final double PIDF_F_SMALL_TRIANGLE = 13.25;
    public final double PIDF_F_BIG_TRIANGLE = 14.0;
    public final double PIDF_F_CLOSE = 13.0;

    public static double shooterP = 650;//150; //was 75change from 25.0
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 350;//25.0;//14.25;

    public static double turretPositionCoefficient = 25.0;
    public static double turretP = 10.0;
    public static double turretI = 0.0;
    public static double turretD = 0.0;
    public static double turretF = 0.0;
    public static double turretF_spinning = 1.0;
    public long autoSleepTime = 1000;
    final float MAX_SHOOTER_RPM = 4500;
    final float MIN_SHOOTER_RPM = 0;
    final double YELLOW_JACKET_19_1_TICKS = 537.7; // 19.2:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_1_1_TICKS = 28.0; // 1:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_3_1_TICKS = 103.8; // 3.7:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_13_1_TICKS = 384.5; // 13.7:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_5_1_TICKS = 145.1; // 5.2:1 - ticks per motor shaft revolution
    final double TURRET_TICKS_IN_DEGREES = (133.0/24.0/360.0) * YELLOW_JACKET_5_1_TICKS; // 133/24 is the gear ratio
    private int turretTicksErrorOffset = 0;
    private final double MAX_TURRET_ANGLE = 145.0;
    private final double MIN_TURRET_ANGLE = -165.0;
    final int MAX_TURRET_TICKS = (int)(MAX_TURRET_ANGLE * TURRET_TICKS_IN_DEGREES);
    final int MIN_TURRET_TICKS = (int)(MIN_TURRET_ANGLE * TURRET_TICKS_IN_DEGREES);
    final double DISTANCE_FROM_FRONT_TO_BACK_OF_TARGET = 18.25;
    private double turretTargetAngle = 0.0;
    //public Servo allianceIndicatorLight = null;
    private String allianceColorString = "UNKNOWN";
    final double INDICATOR_BLACK = 0;
    final double INDICATOR_RED = 0.279;
    final double INDICATOR_BLUE = 0.611;
    final double INDICATOR_WHITE = 1;
    final double INDICATOR_GREEN = 0.500;
    final double INDICATOR_ORANGE = 0.333;
    final double INDICATOR_YELLOW = 0.388;
    final double INDICATOR_SAGE_GREEN = 0.444;
    final double INDICATOR_VIOLET = 0.722;
    final double LAUNCH_RAMP_UP_POSITION = 1;
    final double LAUNCH_RAMP_DOWN_POSITION = 0;
    double indicatorLightValue = 0;
    public final double FEET_TO_METER = 0.3048;
    public final double METER_TO_FEET = 3.28084;
    public final double RADIANS_PER_SECOND_TO_RPM = 9.54929658551; // 60 / (2 * Math.PI)

    public static double aprilTagTargetX = -62.0;
    public static double aprilTagTargetY = -62.0;
    private int currentAprilTargetId = 20;
    GoBildaPinpointDriver pinpoint;
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES,0);
     double distanceToTarget = 0;
    private boolean turretStall = false;
    double stallTimer = 0;

    //CommonAngles and RPM:
    public double FarLaunchRPM= 2950; // References Done, needs tuning
    public double CloseLaunchRPM= 2350.0; // References Done, needs tuning


    public double CloseLaunchHoodAngle=0.5; // References Done, needs tuning
    public double FarLaunchHoodAngle=0.7; // References Done, needs tuning


    public double RedFarLaunchTurretAngle = -120.0;  // was -116 like blue but shot way left // was -122  // was -115 originally
    public double RedCloseLaunchTurretAngle = -136.0;

    public double BlueFarLaunchTurretAngle = 116;    // was 117
    public double BlueCloseLaunchTurretAngle = 132;
    public boolean GamePad2LeftBumper = false;





    DataLogger datalogger;
     private GerickaDatalog logger;


    /// //////////////////////////////////////////////////////////////////

    RevBlinkinLedDriver.BlinkinPattern Blinken_pattern;
    RevBlinkinLedDriver blinkinLedDriver;
    RevColorSensorV3 ColorSensorRight;
    RevColorSensorV3 ColorSensorLeft;

    private int detectedObeliskId = -1;


    DigitalChannel beamBreak1;
    DigitalChannel beamBreak2;
    DigitalChannel beamBreak3;
    DigitalChannel beamBreak4;

    private Servo light1;
    private Servo light2;
    private Servo light3;


    public RobotHardware(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }
    public void Init (HardwareMap hardwareMap, String allianceColor) {
        //allianceIndicatorLight = hardwareMap.get(Servo.class, "IndicatorLight");
        // ************* Drive MOTORS ****************
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront_leftOdometry");
        leftRear= hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear_rightOdometry");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront_centerOdometry");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // GoBilda Pinpoint Odometry computer (reports IMU heading and movement ticks)
        InitPinpoint(hardwareMap);

        // ********** Intake and Shooter System Motors **********************************
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorRight.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        logger = new GerickaDatalog("shearforce_robot_log");

        if (!this.IsDriverControl) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setTargetPosition(0);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setPower(0.0);
        shooterMotorLeft.setPower(0.0);
        shooterMotorRight.setPower(0.0);
        turretMotor.setPower(0.0);

        if (GetShooterPIDF_Enabled()) {
            shooterMotorLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            shooterMotorRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        }

        if (GetTurretPIDF_Enabled()) {
            // PIDF NOTES FOR TURRET:
            // RUN_TO_POSITION actually uses a two-layer control system:
            //     Position Loop: Uses the P-gain (setPositionPIDFCoefficients) to decide how fast to go based on distance.
            //    Velocity Loop: Uses setVelocityPIDFCoefficients to actually achieve that speed.
            //    Tuning: If the motor oscilliates around the target, reduce the P value. If it stops too early, increase it.

            // Set PIDF Coefficients for Position used in RUN_TO_POSITION mode
            // Only Proportional (P) really matters for position control.
            // Higher P = more aggressive, lower P = gentler, too high = oscillation.
            turretMotor.setPositionPIDFCoefficients(turretPositionCoefficient);

            // Tune the underlying velocity PIDF for better movement (how fast it moves)
            turretMotor.setVelocityPIDFCoefficients(turretP, turretI, turretD, turretF);
        }

            // ****************** SERVOS ******************************************
        launchRampServo = hardwareMap.get(Servo.class, "launchRampServo");

        //light1 = hardwareMap.get(Servo.class, "light1");
        //light2 = hardwareMap.get(Servo.class, "light2");
        //light3 = hardwareMap.get(Servo.class, "light3");

        // ********** Color Sensors ********************
        //ColorSensorRight = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        //ColorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");

        if (Objects.equals(allianceColor, "RED")) {
            allianceColorString = "RED";
            //allianceIndicatorLight.setPosition(INDICATOR_RED);
        } else {
            allianceColorString = "BLUE";
            //allianceIndicatorLight.setPosition(INDICATOR_BLUE);
        }

         //limelightbox = hardwareMap.get(Limelight3A.class, "limelight");
        //InitBlinkin(hardwareMap);

        // ***** breakbeam *****
        beamBreak1 =  hardwareMap.get(DigitalChannel.class, "beamBreak1");
        beamBreak2 =  hardwareMap.get(DigitalChannel.class, "beamBreak2");
        beamBreak3 =  hardwareMap.get(DigitalChannel.class, "beamBreak3");
        beamBreak4 =  hardwareMap.get(DigitalChannel.class, "beamBreak4");
        beamBreak1.setMode(DigitalChannel.Mode.INPUT);
        beamBreak2.setMode(DigitalChannel.Mode.INPUT);
        beamBreak3.setMode(DigitalChannel.Mode.INPUT);
        beamBreak4.setMode(DigitalChannel.Mode.INPUT);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();

    }

    public void SetAllianceColor(String value) {
        allianceColorString = value;
    }

    public void ShowTelemetry(){

        //ShowPinpointTelemetry();

        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());

        //opMode.telemetry.addData("Obelisk Id", detectedObeliskId);
        //opMode.telemetry.addData("April Tag Target ID", GetAprilTagTargetId());
        opMode.telemetry.addData("Distance To Target (in.):", distanceToTarget);
        //opMode.telemetry.addData("Obelisk Pattern", obeliskPattern);

        opMode.telemetry.addData("Shooter ", "L-RPM: %.1f, R-RPM: %.1f", CalculateMotorRPM(shooterMotorLeft.getVelocity(), YELLOW_JACKET_1_1_TICKS), CalculateMotorRPM(shooterMotorRight.getVelocity(), YELLOW_JACKET_1_1_TICKS));
        opMode.telemetry.addData("        Target RPM: ", shooterTargetRPM);
        //opMode.telemetry.addData("        ", "L-Vel: %.3f, R-Vel: %.3f" , shooterMotorLeft.getVelocity(), shooterMotorRight.getVelocity());
        //opMode.telemetry.addData("        ", "L-Pow: %.3f, R-Pow: %.3f" , shooterMotorLeft.getPower(), shooterMotorRight.getPower());
        //opMode.telemetry.addData("        ", "L-Amp: %.1f, R-Amp: %.1f" , shooterMotorLeft.getCurrent(CurrentUnit.AMPS), shooterMotorRight.getCurrent(CurrentUnit.AMPS));
        //opMode.telemetry.addData("Shooter Target Speed: ", shooterTargetSpeed);

        opMode.telemetry.addData("Launch Ramp Position", GetLaunchRampPosition());

        opMode.telemetry.addData("Auto Shooter Mode: ", autoShooterMode);
        opMode.telemetry.addData("Auto Turret Mode: ", autoTurretMode);
        opMode.telemetry.addData("Auto Intake Mode: ", autoIntakeMode);
        opMode.telemetry.addData("Auto LaunchRamp Mode: ", GetAutoHoodMode());


        opMode.telemetry.addData("Auto Turret Reset Mode: ", GetTurretAutoResetMode());
        opMode.telemetry.addData("Turret Stall?:", turretStall);
        opMode.telemetry.addData("Turret Motor Is Busy?:", turretMotor.isBusy());
        opMode.telemetry.addData("Turret Velocity:", turretMotor.getVelocity());
        opMode.telemetry.addData("Turret Amps:", turretMotor.getCurrent(CurrentUnit.AMPS));

        opMode.telemetry.addData("Turret ", "ticks: %d, MAX: %d, MIN: %d", turretMotor.getCurrentPosition(), MAX_TURRET_TICKS, MIN_TURRET_TICKS);
        opMode.telemetry.addData("       ", "currentAngle: %.1f, tgt-Angle: %.1f", getCurrentTurretAngle(), turretTargetAngle);
        opMode.telemetry.addData("       Error Offset:", turretTicksErrorOffset);
        //opMode.telemetry.addData("       ", "Pow: %.1f, Amp: %.1f", turretMotor.getPower(),turretMotor.getCurrent(CurrentUnit.AMPS));

        //opMode.telemetry.addData("Intake ", "Pow: %.1f, Amp: %.1f", intakeMotor.getPower(), intakeMotor.getCurrent(CurrentUnit.AMPS));
        //opMode.telemetry.addData("       ", "Vel: %.1f", intakeMotor.getVelocity());
        opMode.telemetry.addData("beamBreak1", !beamBreak1.getState());
        opMode.telemetry.addData("beamBreak2", !beamBreak2.getState());
        opMode.telemetry.addData("beamBreak3", !beamBreak3.getState());
        opMode.telemetry.addData("beamBreak4", !beamBreak4.getState());

        ShowPIDF_Telemetry();
        ShowTurretPIDF_Telemetry();
        //opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());
        //opMode.telemetry.addData("imu roll: ", (imu.getRobotYawPitchRollAngles().getRoll()));
        //opMode.telemetry.addData("imu pitch: ", (imu.getRobotYawPitchRollAngles().getPitch()));
        //opMode.telemetry.addData("imu yaw: ", (imu.getRobotYawPitchRollAngles().getYaw()));
        opMode.telemetry.addData("Alliance: ", allianceColorString);

        telemetryAprilTag();

        //opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);

        opMode.telemetry.addData("Yaw Scalar: ", pinpoint.getYawScalar());
        Write_Datalog();

        opMode.telemetry.update();
    }
    public void Write_Datalog(){
       logger.imuHeading.set(GetIMU_HeadingInDegrees());
        //logger.resetPositionCount.set(resetPositionCount);
        logger.distanceToTarget.set(distanceToTarget);
        logger.LaunchRampPosition.set(GetLaunchRampPosition());
        logger.shooterRPM.set(CalculateMotorRPM(shooterMotorLeft.getVelocity(), YELLOW_JACKET_1_1_TICKS));
        logger.shooterTargetRPM.set(shooterTargetRPM);
        logger.YawScalar.set(pinpoint.getYawScalar());
        logger.turretTicks.set(turretMotor.getCurrentPosition());
        logger.turrentAngle.set(getCurrentTurretAngle());
       logger.writeLine();
    }
    public void ShowPIDF_Telemetry() {
        PIDFCoefficients shooterPIDF_Left = shooterMotorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.telemetry.addData("ShooterL PIDF: ", "p: %.3f  i: %.2f  d: %.2f  f: %.2f", shooterPIDF_Left.p, shooterPIDF_Left.i, shooterPIDF_Left.d, shooterPIDF_Left.f);

        PIDFCoefficients shooterPIDF_Right = shooterMotorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.telemetry.addData("ShooterR PIDF: ", "p: %.3f  i: %.2f  d: %.2f  f: %.2f", shooterPIDF_Right.p, shooterPIDF_Right.i, shooterPIDF_Right.d, shooterPIDF_Right.f);

    }
    
    public void SetShooterPIDFCoefficients() {
        if (GetShooterPIDF_Enabled()) {
            PIDFCoefficients shooterPIDF = shooterMotorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            // if the static PIDF coefficients changed (probably through the dashboard)
            if ((shooterPIDF.p != shooterP) || (shooterPIDF.i != shooterI) || (shooterPIDF.d != shooterD) || (shooterPIDF.f != shooterF)) {
                shooterMotorLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
                shooterMotorRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            }
        }
    }

    public boolean GetShooterPIDF_Enabled() { return shooterPIDF_Enabled; }
    public void SetShooterPIDF_Enabled(boolean value) { shooterPIDF_Enabled = value; }


    public void EndgameBuzzer(){
        boolean invalidHeading = false;
        double turretBearing = calculateBearingToAprilTag();

        boolean beam2IsBroken = !beamBreak2.getState(); // Invert if 'true' means broken
        boolean beam3IsBroken = !beamBreak3.getState();
        boolean beam4IsBroken = !beamBreak4.getState();
        boolean redLightOn = false;

        if ((beam4IsBroken)  &&(!beam2IsBroken)){ //&& (!beam3IsBroken)
            redLightOn = true;
        }

        // check if it is possible for the turret to point at the target
        if ((turretBearing > MAX_TURRET_ANGLE) || (turretBearing < MIN_TURRET_ANGLE))
        {
            invalidHeading = true;
        }

        if (invalidHeading || redLightOn) {
            if (!opMode.gamepad1.isRumbling()) { // Check for possible overlap of rumbles.
                opMode.gamepad1.rumble(500);
            }
        }
        else if(opMode.getRuntime() < 110.5 && opMode.getRuntime() > 110.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
        else if(opMode.getRuntime() < 100.5 && opMode.getRuntime() > 100){
            opMode.gamepad1.rumble(500);
            opMode.gamepad2.rumble(500);
        }
    }

    // *************************************************************************
    //      Pinpoint Utility Functions
    // *************************************************************************

    public void InitPinpoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        // Xpod is -84.0 MM from pinpoint computer, and -109.5 mm from center point of robot
        // Ypod is aligned with pinpoint computer in X, and -198.0 mm behind the center point of robot
        double xOffset = -109.5; //mm - X pod is to the right of pinpoint and center when viewed from the top
        double yOffset = -198; // mm - Y pod is -198 mm behind the center rotation point of chassis
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        // need to verify X,Y directions, going forward should make X count up, going Left should make Y count up
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD
                , GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.setYawScalar(yawScalar);
        pinpoint.resetPosAndIMU();
        //pinpoint.recalibrateIMU();

        pinpoint.update();
    }

    public void ShowPinpointTelemetry() {
        if (pinpoint != null) {
            pinpoint.update();

        /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
        */
            opMode.telemetry.addData("pinpoint status: ", pinpoint.getDeviceStatus());
            opMode.telemetry.addData("pinpoint ", "x-ticks: %d, y-ticks: %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
            opMode.telemetry.addData("pinpoint Relative Pos(inches): ", "x: %.1f, y: %.1f", pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint Relative Position(inches): ", "x: %.1f, y: %.1f", pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint Relative Heading(deg): ", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
            opMode.telemetry.addData("pinpoint x velocity",pinpoint.getVelX(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint y velocity",pinpoint.getVelY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint velocity heading:",pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            Pose2D currentPosition = GetPinpointAbsolutePosition(startPose);
            opMode.telemetry.addData("pinpoint Absolute Position(inches): ", "x: %.1f, y: %.1f", currentPosition.getX(DistanceUnit.INCH), currentPosition.getY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint Absolute Heading(deg): ", currentPosition.getHeading(AngleUnit.DEGREES));

        }

    }


    public Pose2D GetPinpointAbsolutePosition(Pose2D pos){
        double AbsoluteX = pinpoint.getPosY(DistanceUnit.INCH) + pos.getX(DistanceUnit.INCH);
        double AbsoluteY = pinpoint.getPosX(DistanceUnit.INCH) + pos.getY(DistanceUnit.INCH);
        double AbsoluteAngle = pinpoint.getHeading(AngleUnit.DEGREES) - pos.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, AbsoluteX, AbsoluteY,AngleUnit.DEGREES, AbsoluteAngle);
    }

    // *************************************************************************
    //      Webcam & April Tag Utility Functions
    // *************************************************************************

   public void WebcamInit (HardwareMap hardwareMap){
       // Create the AprilTag processor.
       aprilTag = new AprilTagProcessor.Builder()

               // The following default settings are available to un-comment and edit as needed.
               //.setDrawAxes(true) // Changed in V12.0
               //.setDrawTagOutline(true)
               //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
               //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
               //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
               //.setDrawCubeProjection(false)

               // == CAMERA CALIBRATION ==
               // If you do not manually specify calibration parameters, the SDK will attempt
               // to load a predefined calibration for your camera.
               //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
               // ... these parameters are fx, fy, cx, cy.

               .build();

       // Adjust Image Decimation to trade-off detection-range for detection-rate.
       // eg: Some typical detection data using a Logitech C920 WebCam
       // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
       // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
       // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
       // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
       // Note: Decimation can be changed on-the-fly to adapt during a match.
       //aprilTag.setDecimation(3);

       // Create the vision portal by using a builder.
       VisionPortal.Builder builder = new VisionPortal.Builder();

       builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

       // Choose a camera resolution. Not all cameras support all resolutions.
       //builder.setCameraResolution(new Size(640, 480));

       // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
       //builder.enableLiveView(true);

       // Set the stream format; MJPEG uses less bandwidth than default YUY2.
       //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

       // Choose whether or not LiveView stops if no processors are enabled.
       // If set "true", monitor shows solid orange screen if no processors enabled.
       // If set "false", monitor shows camera view without annotations.
       //builder.setAutoStopLiveView(false);

       // Set and enable the processor.
       builder.addProcessor(aprilTag);

       // Build the Vision Portal, using the above settings.
       visionPortal = builder.build();

       // Disable or re-enable the aprilTag processor at any time.
       //visionPortal.setProcessorEnabled(aprilTag, true);

   }
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        opMode.telemetry.addData("Webcam Tags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    opMode.telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    opMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    opMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                opMode.telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                opMode.telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                opMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                opMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                opMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }

    }
    public void updateObeliskFromCamera(){
        if(detectedObeliskId == -1){
            int id = readObeliskId();

            if(id != -1){
                detectedObeliskId = id;
            }
        }
        else{
            readObeliskId();
        }
    }

    public int getDetectedObeliskId(){
        return detectedObeliskId;
    }

    public int detectObeliskMotif(long timeoutMs){
        detectedObeliskId = -1; // here we will reset the latched value
        long startTime = System.currentTimeMillis();

        while(!opMode.isStopRequested() && detectedObeliskId == -1 && System.currentTimeMillis() - startTime < timeoutMs){
            updateObeliskFromCamera(); // uses the method here
            ShowTelemetry(); // ask if this logic is fine????
            opMode.sleep(20); // i don't want to spam our telemetry
        }
        return detectedObeliskId; // it will be -1 if we never saw one
    }

    private int readObeliskId() {

        if (getObeliskTagVisible(21)) {
            detectedTagId = 21;
            return 21;
        }
        if (getObeliskTagVisible(22)) {
            detectedTagId = 22;
            return 22;
        }
        if (getObeliskTagVisible(23)) {
            detectedTagId = 23;
            return 23;
        }

        if (getAprilTagVisible(23) && getAprilTagVisible(20)) {
            detectedTagId = 23;
            return 23;
        }


        // cannot see anythign
        detectedTagId = -1;
        return -1;
    }


    double getBearingToAprilTag(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;
        double bearing = 0.0;

        for (AprilTagDetection detection : detections){
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if(singleDet.id == detectionID){
                    bearing = detection.ftcPose.bearing;
                    foundit = true;
                    break;
                }
            }
        }

        return bearing;
    }

    double getBearingToObeliskTag(int detectionID){

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.id == detectionID) {
                    return detection.ftcPose.bearing;
                }
            }
        }


        return 0.0;
    }

    double getDistanceToObeliskTag(int detectionID){

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.id == detectionID) {
                    return detection.ftcPose.range;
                }
            }
        }

        return 0.0;
    }

    boolean getAprilTagVisible(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.id == detectionID) {
                    foundit = true;
                    break; // this will break after first item with metadata(DO NOT USE FOR OBELISK)
                }
            }
        }

        return foundit;
    }

    boolean getObeliskTagVisible(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.id == detectionID) {
                   return true;
                }
            }
        }

        return false;
    }

    double getDistanceToAprilTag(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;
        double distance = 0.0;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.id == detectionID) {
                    distance = detection.ftcPose.range;
                    break; // this will break after first item with metadata(DO NOT USE FOR OBELISK)
                }
            }
        }


        return distance;
    }

    public void SetAprilTagTargetId(int value) {
        currentAprilTargetId = value;
        if (currentAprilTargetId == 24) {  // RED
           // aprilTagTargetX = TARGET_X_RED;
           // aprilTagTargetY = TARGET_Y_RED;
        }
        else if (currentAprilTargetId == 20) { // BLUE
            //aprilTagTargetX = TARGET_X_BLUE;
           // aprilTagTargetY = TARGET_Y_BLUE;
        }
    }
    public int GetAprilTagTargetId() { return currentAprilTargetId; }




    // *********************************************************
    // ****      LED Lights Controls and Utility functions
    // *********************************************************

    public void SetIndicatorLights() {

        boolean invalidHeading = false;
        double turretBearing = calculateBearingToAprilTag();
        // check if it is possible for the turret to point at the target
        if ((turretBearing > MAX_TURRET_ANGLE) || (turretBearing < MIN_TURRET_ANGLE))
        {
            invalidHeading = true;
        }
        boolean beam2IsBroken = !beamBreak2.getState(); // Invert if 'true' means broken
        boolean beam3IsBroken = !beamBreak3.getState();
        boolean beam4IsBroken = !beamBreak4.getState();
        boolean turretReady = ((turretMotor.getCurrentPosition() > (turretMotor.getTargetPosition() - 5)) && (turretMotor.getCurrentPosition() < (turretMotor.getTargetPosition() + 5)));  // 5 ticks is a little bit more than 2 degrees
        double currentMotorRPM = CalculateMotorRPM(shooterMotorLeft.getVelocity(), YELLOW_JACKET_1_1_TICKS);
        boolean shooterReady =  ((currentMotorRPM >= shooterTargetRPM - 50) && (currentMotorRPM <= shooterTargetRPM + 200));

        // **** INDICATOR LIGHT #1              ************
        if (beam2IsBroken) {
            light1.setPosition(INDICATOR_GREEN);
        }
        else {
            light1.setPosition(INDICATOR_BLACK);
        }

        // **** INDICATOR LIGHT #2              ************
        /*if (beam3IsBroken){
            light2.setPosition(INDICATOR_GREEN);
        }
        else {
            light2.setPosition(INDICATOR_BLACK);
        }*/

        // **** INDICATOR LIGHT #3              ************
        if ((beam4IsBroken)  &&(!beam2IsBroken)){ // && (!beam3IsBroken)
            light3.setPosition(INDICATOR_RED);
        }
        else if (beam4IsBroken){
            light3.setPosition(INDICATOR_GREEN);
        }
        else {
            light3.setPosition(INDICATOR_BLACK);
        }
    }
    public void InitBlinkin(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class,"RevBLinkinLedDriver");
        /*
        if(allianceColorIsBlue)
        {
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        else
        {
            Blinken_pattern  = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        */

        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriver.setPattern(Blinken_pattern);
    }

    // *************************************************************************
    //      Shooter (spinning wheels) functions
    // *************************************************************************

    public double CalculateInitialVelocity(double distanceToGoalInMeters){
        double speedInmetersPerSecond = 0.0;
        double topPart = 0.0;
        double bottomPart = 0.0;
        topPart = 9.81 * distanceToGoalInMeters;
        bottomPart = 2 * Math.pow(Math.cos(59),2) * (distanceToGoalInMeters * Math.tan(59) + 0.98425 - 0.4064 + 0.127);
        speedInmetersPerSecond = Math.sqrt(topPart/bottomPart);
        return speedInmetersPerSecond;
        /*
        9.81 is gravitational force
        22.5 is angle of launch
        0.98425 - 0.4064 + 0.127 is height of goal - height of launch + minimal launch distance
        https://www.desmos.com/calculator/n9syawsgk2
         */
    }
    public double CalculateInitialWheelRotationalVelocity(double initialVelocityInMetersPerSecond){
        double initalWheelRotationalVelocityInRadiansPerSecond = 0.0;
        double insideParenthesis = 0.0;
        double outsideParenthesis = 0.0;
        insideParenthesis = 1 + ((1 + 0.4) / 4 * 0.5) * 0.0748 / 0.285;
        outsideParenthesis = 2 * initialVelocityInMetersPerSecond / 0.0508;
        initalWheelRotationalVelocityInRadiansPerSecond = insideParenthesis * outsideParenthesis;
        /*
        The 0.4 is a estimation for the shape factor of the projectile.
        The 0.5 is a estimation for the shape factor of the wheel launcher.
        The 0.0748 and 0.0185 are a good estimate for the mass of the projectile and the mass of the wheel in that order.
        The last value in outsideParenthesis is the radius of the wheel.
         */
        return initalWheelRotationalVelocityInRadiansPerSecond;
    }
    public void SetAutoShooterMode(boolean value){
        autoShooterMode = value;
    }
    public boolean GetAutoShooterMode() { return autoShooterMode; }
    public void SetShooterRPMFromWebCam(){

        if (getAprilTagVisible(GetAprilTagTargetId())){
            double distance = getDistanceToAprilTag(GetAprilTagTargetId());
            if (GetAutoHoodMode()) {
                SetLaunchRampPosition(CalculateOptimumLaunchRampPosition(distance));
            }
            SetShooterMotorToSpecificRPM(CalculateOptimumShooterRPM(distance));
            //currentAprilTargetId = currentTargetId;
        }
    }
    public double CalculateOptimumLaunchRampPosition(double distanceInInches) {
        double optimumLaunchPosition = 0;

        if      (distanceInInches >= 126) { optimumLaunchPosition = 0.7; }  // small triangle position
        else if (distanceInInches >= 120) { optimumLaunchPosition = 0.7; }
        else if (distanceInInches >= 114) { optimumLaunchPosition = 0.7; }
        else if (distanceInInches >= 108) { optimumLaunchPosition = 0.7; }
        else if (distanceInInches >= 102) { optimumLaunchPosition = 0.7; }
        else if (distanceInInches >= 96)  { optimumLaunchPosition = 0.6; }
        else if (distanceInInches >= 90)  { optimumLaunchPosition = 0.6; }
        else if (distanceInInches >= 84)  { optimumLaunchPosition = 0.6; } // tip of big triangle
        else if (distanceInInches >= 78)  { optimumLaunchPosition = 0.6; }
        else if (distanceInInches >= 72)  { optimumLaunchPosition = 0.6; }
        else if (distanceInInches >= 66)  { optimumLaunchPosition = 0.6; }
        else if (distanceInInches >= 60)  { optimumLaunchPosition = 0.5; }
        else if (distanceInInches >= 54)  { optimumLaunchPosition = 0.4; }
        else if (distanceInInches >= 48)  { optimumLaunchPosition = 0.4; }
        else if (distanceInInches >= 42)  { optimumLaunchPosition = 0.2; } // frooty loops spot
        else if (distanceInInches >= 36)  { optimumLaunchPosition = 0.2; }
        else if (distanceInInches >= 30)  { optimumLaunchPosition = 0.1; }
        else if (distanceInInches >= 24)  { optimumLaunchPosition = 0.0; } // super close
        else if (distanceInInches >= 18)  { optimumLaunchPosition = 0.0; }
        else if (distanceInInches >= 12)  { optimumLaunchPosition = 0.0; }
        else {
            optimumLaunchPosition = 0.0;
        }

        return optimumLaunchPosition;
    }
    public double CalculateOptimumShooterRPM(double distanceInInches){
        double optimumShooterRPM = 0;
        double distanceAboveLower = 0.0;
        double rpmDifferenceInRange = 0.0;
        double differenceInMeasurements = 6.0; // inches
        if (distanceInInches >= 126) { optimumShooterRPM = 3150; }
        else if (distanceInInches >= 120) {
            distanceAboveLower = distanceInInches - 120;
            rpmDifferenceInRange = 3000 - 2950;
            optimumShooterRPM = 2950 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 114) {
            distanceAboveLower = distanceInInches - 114;
            rpmDifferenceInRange = 2950 - 2900;
            optimumShooterRPM = 2900 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 108) {
            distanceAboveLower = distanceInInches - 108;
            rpmDifferenceInRange = 2900 - 2900;
            optimumShooterRPM = 2900 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 102) {
            distanceAboveLower = distanceInInches - 102;
            rpmDifferenceInRange = 2900 - 2850;
            optimumShooterRPM = 2850 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 96) {
            distanceAboveLower = distanceInInches - 96;
            rpmDifferenceInRange = 2850 - 2750;
            optimumShooterRPM = 2750 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 90) {
            distanceAboveLower = distanceInInches - 90;
            rpmDifferenceInRange = 2750 - 2700;
            optimumShooterRPM = 2700 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 84) {
            distanceAboveLower = distanceInInches - 84;
            rpmDifferenceInRange = 2700 - 2650;
            optimumShooterRPM = 2650 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 78) {
            distanceAboveLower = distanceInInches - 78;
            rpmDifferenceInRange = 2650 - 2600;
            optimumShooterRPM = 2600 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 72) {
            distanceAboveLower = distanceInInches - 72;
            rpmDifferenceInRange = 2600 - 2500;
            optimumShooterRPM = 2500 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 66) {
            distanceAboveLower = distanceInInches - 66;
            rpmDifferenceInRange = 2500 - 2500;
            optimumShooterRPM = 2500 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 60) {
            distanceAboveLower = distanceInInches - 60;
            rpmDifferenceInRange = 2500 - 2400;
            optimumShooterRPM = 2400 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 54) {
            distanceAboveLower = distanceInInches - 54;
            rpmDifferenceInRange = 2400 - 2300;
            optimumShooterRPM = 2300 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 48) {
            distanceAboveLower = distanceInInches - 48;
            rpmDifferenceInRange = 2300 - 2300;
            optimumShooterRPM = 2300 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 42) {
            distanceAboveLower = distanceInInches - 42;
            rpmDifferenceInRange = 2300 - 2200;
            optimumShooterRPM = 2200 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 36) {
            distanceAboveLower = distanceInInches - 36;
            rpmDifferenceInRange = 2200 - 2100;
            optimumShooterRPM = 2100 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 30) {
            distanceAboveLower = distanceInInches - 30;
            rpmDifferenceInRange = 2100 - 2000;
            optimumShooterRPM = 2000 + (distanceAboveLower / differenceInMeasurements) * rpmDifferenceInRange;
        }
        else if (distanceInInches >= 24) {
            optimumShooterRPM = 2000;
        }
        else if (distanceInInches >= 18) {
            optimumShooterRPM = 2000;
        }
        else if (distanceInInches >= 12) {
            optimumShooterRPM = 2000;
        }
        else {
            optimumShooterRPM = 2000;
        }

        // calculate PIDF values
        if (distanceInInches > 108) {
            shooterF = PIDF_F_SMALL_TRIANGLE;  // 3400 to 3500 = 13.25
        }
        else if (distanceInInches > 66)
        {
            shooterF = PIDF_F_BIG_TRIANGLE; // 2900 to 3000 = 14.0
        }
        else if (distanceInInches > 32)
        {
            //shooterF = PIDF_F_FROOTY_LOOPS; // 2300 to 2350 = 14.25
        }
        else {
            shooterF = PIDF_F_CLOSE; // 2300 = 13.0
        }


        /*double testDistance = 24;
        while (Math.min(distanceInInches,testDistance) != distanceInInches){
            if (testDistance != 84) {
                testDistance += 6;
            }
            else {
                testDistance = 120;
                break;
            }
        }
        if      (testDistance == 24){  optimumShooterRPM = 2100; }
        else if (testDistance == 30){  optimumShooterRPM = 2200; }
        else if (testDistance == 36){  optimumShooterRPM = 2300; }
        else if (testDistance == 42){  optimumShooterRPM = 2350; }
        else if (testDistance == 48){  optimumShooterRPM = 2350; }
        else if (testDistance == 54){  optimumShooterRPM = 2400; }
        else if (testDistance == 60){  optimumShooterRPM = 2650; }
        else if (testDistance == 66){  optimumShooterRPM = 2750; }
        else if (testDistance == 72){  optimumShooterRPM = 2800; }
        else if (testDistance == 78){  optimumShooterRPM = 2950; }
        else if (testDistance == 120){ optimumShooterRPM = 3500; }
        else {
            optimumShooterRPM = 2500;
        }*/
        return optimumShooterRPM;


        /*  Original Measured values:
            24 inch - 2100rpm
            30 inch - 2200rpm
            36 inch - 2300rpm
            42 inch - 2350rpm
            48 inch - 2350rpm
            54 inch - 2400rpm
            60 inch - 2650rpm
            66 inch - 2750rpm
            72 inch - 2800rpm
            78 inch - 2950rpm
            far launch zone (120 inch) - 3500rpm

            24, 80(tip of large triangle), 120 inch for tip of small triangle

        */

    }
    double CalculateMotorRPM (double motorVelocity, double motorCountsPerRevolution)
    {
        return (motorVelocity / motorCountsPerRevolution) * 60.0;
    }

    public void SetShooterMotorToSpecificRPM(double desiredRPM){
        shooterTargetRPM = desiredRPM;
        shooterTargetRPM = Math.min(shooterTargetRPM,MAX_SHOOTER_RPM);
        shooterTargetRPM = Math.max(shooterTargetRPM, MIN_SHOOTER_RPM);
        shooterMotorLeft.setVelocity(shooterTargetRPM * YELLOW_JACKET_1_1_TICKS / 60);
        shooterMotorRight.setVelocity(shooterTargetRPM * YELLOW_JACKET_1_1_TICKS / 60);
    }

    public double GetShooterTargetRPM() { return shooterTargetRPM; }
    private double CalculateDistanceToPoint(double currentXInInches, double currentYInInches, double targetXInInches, double targetYInInches) {
        double deltaX = targetXInInches - currentXInInches;
        double deltaY = targetYInInches - currentYInInches;

        return Math.hypot(deltaX, deltaY);
    }

/*    public void CalculateDistanceToTarget() {
        double targetDistance = CalculateDistanceToPoint(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, aprilTagTargetX, aprilTagTargetY);
        distanceToTarget = targetDistance - DISTANCE_FROM_FRONT_TO_BACK_OF_TARGET;
    }
    public void SetShooterRPMFromRoadrunner(){

        drive.updatePoseEstimate();

        CalculateDistanceToTarget();

        if (!GetUseOnlyWebcamForDistance() && GetAutoShooterMode()) {
            if (GetAutoHoodMode()) {
                SetLaunchRampPosition(CalculateOptimumLaunchRampPosition(distanceToTarget));
            }
            SetShooterMotorToSpecificRPM(CalculateOptimumShooterRPM(distanceToTarget));
        }
    }

 */

    // *************************************************************************
    //      Intake Functions
    // *************************************************************************

    public boolean GetAutoIntakeMode() { return autoIntakeMode; }
    public void SetAutoIntakeMode(boolean value) { autoIntakeMode = value; }
    public void SetIntakeMotor(boolean on,boolean intake){
        if (on && intake) {
            intakeMotor.setPower(INTAKE_POWER);
        }
        else if (!on){
            intakeMotor.setPower(0);
        }
        else {
            intakeMotor.setPower(-INTAKE_POWER);
        }
    }


    // *************************************************************************
    //      Rotating Turret Functions
    // *************************************************************************
    public void ShowTurretPIDF_Telemetry() {
        PIDFCoefficients turretPIDF = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients turretPositionPIDF = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        opMode.telemetry.addData("Turret PIDF: ", "PositionP: %.3f VelP: %.3f  i: %.2f  d: %.2f  f: %.2f", turretPositionPIDF.p, turretPIDF.p, turretPIDF.i, turretPIDF.d, turretPIDF.f);
    }

    public void SetTurretPIDFCoefficients() {

        if (GetTurretPIDF_Enabled()) {
            PIDFCoefficients turretPIDF = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients turretPositionPIDF = turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            // if the static PIDF coefficients changed (probably through the dashboard)
            if ((turretPositionPIDF.p != turretPositionCoefficient) || (turretPIDF.p != turretP) || (turretPIDF.i != turretI) || (turretPIDF.d != turretD) || (turretPIDF.f != turretF)) {
                // PIDF NOTES FOR TURRET:
                // RUN_TO_POSITION actually uses a two-layer control system:
                //    Position Loop: Uses the P-gain (setPositionPIDFCoefficients) to decide how fast to go based on distance.
                //    Velocity Loop: Uses setVelocityPIDFCoefficients to actually achieve that speed.
                //    Tuning: If the motor oscilliates around the target, reduce the P value. If it stops too early, increase it.

                // Set PIDF Coefficients for Position used in RUN_TO_POSITION mode
                // Only Proportional (P) really matters for position control.
                // Higher P = more aggressive, lower P = gentler, too high = oscillation.
                turretMotor.setPositionPIDFCoefficients(turretPositionCoefficient);

                // Tune the underlying velocity PIDF for better movement (how fast it moves)
                turretMotor.setVelocityPIDFCoefficients(turretP, turretI, turretD, turretF);
            }
        }
    }

    public boolean GetTurretPIDF_Enabled() { return turretPIDF_Enabled; }

    public void SetTurretRotationAngle(double degrees){
        // normalize the angle to be -180 to +180
        if (autoTurretMode){
            while (degrees > 180) degrees -= 360;
            while (degrees < -180) degrees += 360;
        }
        turretTargetAngle = degrees;

        if (autoTurretMode) {
            // if turretTargetAngle is in the dead-zone
            if ((turretTargetAngle <= MIN_TURRET_ANGLE) || (turretTargetAngle >= MAX_TURRET_ANGLE)) {
                // Keep the turret on the same side until robot moves into a valid shooting position
                // as opposed to swinging into the other extreme position and still being invalid
                if (getCurrentTurretAngle() < -90) {
                    turretTargetAngle = MIN_TURRET_ANGLE;
                }
                else if (getCurrentTurretAngle() > 90.0) {
                    turretTargetAngle = MAX_TURRET_ANGLE;
                }
                else {
                    turretTargetAngle = Math.min(turretTargetAngle, MAX_TURRET_ANGLE);
                    turretTargetAngle = Math.max(turretTargetAngle, MIN_TURRET_ANGLE);
                }
            }
        }

        // Calculate the correct ticks to achieve this angle
        int turretTargetTicks = (int)(turretTargetAngle * TURRET_TICKS_IN_DEGREES) + turretTicksErrorOffset;

        if (autoTurretMode) {
            turretTargetTicks = Math.min(turretTargetTicks, MAX_TURRET_TICKS + turretTicksErrorOffset);
            turretTargetTicks = Math.max(turretTargetTicks, MIN_TURRET_TICKS + turretTicksErrorOffset);
        }

        // Command the turret motor to go to that position
        turretMotor.setTargetPosition(turretTargetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Limit the speed / velocity of the turret movement based on how far we have to move
        double differenceInAngles = turretTargetAngle - getCurrentTurretAngle();
        // if more than 90 degrees of movement needed
        if (Math.abs(differenceInAngles) > 90){
            turretMotor.setPower(1);
        }
        // if less than 90 degrees of movement needed, but moving to a Limit position
        else if (turretTargetAngle == MAX_TURRET_ANGLE || turretTargetAngle == MIN_TURRET_ANGLE){
            turretMotor.setPower(0.5);
        }
        // else moving less than 90 degrees, but it should be safe
        else{
            turretMotor.setPower(1.0);
        }
    }
    double getTurretTargetAngle(){
        return turretTargetAngle;
    }
    double getCurrentTurretAngle (){
        return (turretMotor.getCurrentPosition() + turretTicksErrorOffset)/TURRET_TICKS_IN_DEGREES;
    }

    public void TurnOffTurret() {
        turretMotor.setPower(0.0);
    }

    public boolean GetTurretAutoMode() { return autoTurretMode; }
    public void SetTurretAutoMode(boolean value) { autoTurretMode = value; }
    void adjustTurretToTargetAprilTag(){
        double bearingToAprilTag = 0.0;
        double currentTurretAngle = getCurrentTurretAngle();
        // normalize the turret angle to be -180 to +180
        while (currentTurretAngle > 180) currentTurretAngle -= 360;
        while (currentTurretAngle < -180) currentTurretAngle += 360;
        // else can't see the april tag, try using the roadrunner position and robot heading to calculate the turret angle
        if (getAprilTagVisible(currentAprilTargetId)){
            bearingToAprilTag = getBearingToAprilTag(currentAprilTargetId);
            SetTurretRotationAngle(bearingToAprilTag + currentTurretAngle);
        }
    }

    private double calculateBearingToAprilTag() {
        // calculate the bearing from the front of the robot to the april tag
        double bearingToAprilTag = 0.0; //calculateBearingToPointInDegrees(drive.localizer.getPose().position.x , drive.localizer.getPose().position.y, aprilTagTargetX, aprilTagTargetY, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        // convert that bearing into a turret angle (turret zero position points the opposite direction of the robot)
        bearingToAprilTag += 180;
        // normalize the bearing to be -180 to +180
        while (bearingToAprilTag > 180) bearingToAprilTag -= 360;
        while (bearingToAprilTag < -180) bearingToAprilTag += 360;

        return bearingToAprilTag;
    }
    public static boolean turret_use_rotation_velocity = false;
    public static boolean turret_shoot_while_moving_enabled = false;

    public double calculateBearingToPointInDegrees(double robotXInInches, double robotYInInches, double targetXInInches, double targetYInInches, double robotHeadingDegrees) {
        double calculatedAngle = 0.0;

        double deltaX = targetXInInches - robotXInInches;
        double deltaY = targetYInInches - robotYInInches;
        double angleToTargetDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Calculate the robot's angular velocity from RoadRunner
        //PoseVelocity2d currentVelocity = drive.updatePoseEstimate();
        //double robotAngularVelocity = currentVelocity.angVel;


        calculatedAngle = angleToTargetDeg - robotHeadingDegrees;
        turretF = 0.0;


        return calculatedAngle;
    }

    public void checkTurretLimit(){
        if (turretMotor.getVelocity() == 0 && turretMotor.isBusy() && turretTargetAngle < getCurrentTurretAngle() && turretMotor.getCurrent(CurrentUnit.AMPS) > 1.0){
            if (stallTimer == 0){
                stallTimer = opMode.getRuntime();
            }
            if (opMode.getRuntime() > 0.3 + stallTimer) {
                turretStall = true;

            } else {
                turretStall = false;
            }
        }
        else {
            stallTimer = 0;
            turretStall = false;
        }
    }

    public boolean GetTurretAutoResetMode() { return autoTurretResetMode; }
    public void SetTurretAutoResetMode(boolean value) { autoTurretResetMode = value; }
    public void autoTurretReset() {
        if (autoTurretResetMode){
            if (turretStall) {
                turretTicksErrorOffset = turretMotor.getCurrentPosition() - -380;
                turretStall = false;
                stallTimer = 0;
            }
        }
    }

    public void resetTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretTargetAngle = 0.0;
        turretMotor.setTargetPosition(0);
        SpecialSleep(50);
        SetTurretRotationAngle(0.0);
        turretTicksErrorOffset = 0;
    }


    // *************************************************************************
    //      Launch Ramp Functions
    // *************************************************************************
    public void SetLaunchRampPosition(double position){
        double launchRampTargetPosition = 0;
        launchRampTargetPosition = Math.min(position,LAUNCH_RAMP_UP_POSITION);
        launchRampTargetPosition = Math.max(position,LAUNCH_RAMP_DOWN_POSITION);
        launchRampServo.setPosition(launchRampTargetPosition);
    }
    public double GetLaunchRampPosition(){
        return launchRampServo.getPosition();
    }
    public boolean GetAutoHoodMode(){return autoHoodMode;}
    public void SetAutoHoodMode(boolean value) { autoHoodMode = value;}
    // *************************************************************************
    //      Drive Control Functions
    // *************************************************************************

    public void driveControlsRobotCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x * 1.1;
        double rx = opMode.gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public double GetIMU_HeadingInDegrees()
    {
        return AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuOffsetInDegrees);

    }
    public void driveControlsFieldCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x;
        double rx = opMode.gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
    public void RunDriveControls() {
        if (IsFieldCentric) {
            driveControlsFieldCentric();
        }
        else {
            driveControlsRobotCentric();
        }
    }
    public void SetFieldCentricMode(boolean fieldCentricEnabled) {
        IsFieldCentric = fieldCentricEnabled;
    }
    public void SpecialSleep(long milliseconds) {
        for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(milliseconds); stop > System.nanoTime(); ) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;
            if (IsDriverControl) {
                if (IsFieldCentric) driveControlsFieldCentric();
                if (!IsFieldCentric) driveControlsRobotCentric();
            }
            //if(pidfSlidesEnabled || pidfEnabled){
            //    opMode.sleep(5);
            //}
        }
    }

}