package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// auto select manual opMode next
@Disabled
@Autonomous(name="Blue Far Auto With Loading Zone", preselectTeleOp ="Gericka 1 Manual Control")
public class Blue_Far_Auto_With_Loading_Zone extends LinearOpMode {
    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    // Trajectories
    Action DriveToLoadingZoneAndBack;
    Action ReleaseGateFromFirstMark;
    Action ReleaseGateFromSecondMark;
    Action ReleaseGateFromThirdMark;
    Action GateToSmallTriangle;
    Action DriveToLoadingZone;
    Action DriveLoadingZoneToSmallTriangle;
    Action DriveToSecondMark;
    Action DriveToShootingPosition;
    Action DriveSecondMarkToSmallTriangle;
    Action DriveToFirstMark;
    Action DriveFirstMarkToSmallTriangle;
    Action DriveOutOfLaunchZone;
    Action DriveToThirdMark;
    int lifterUpSleepTime = 500;
    int lifterDownSleepTime = 600;

    public void runOpMode() {
        startPose = new Pose2d(60, -12, Math.toRadians(-90));
        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "BLUE");

        // initialize roadrunner
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);
        theRobot.InitRoadRunner(drive);


        // initialize the webcam
        theRobot.WebcamInit(this.hardwareMap);

        sleep(500); // sleep at least 1/4 second to allow pinpoint to calibrate itself
        // finish initializing the pinpoint
        theRobot.SetRoadrunnerInitialPosition(60, -12, -90);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "BLUE");

        // set lifter half up (so can get 3 ball loaded in robot)
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);

        //control.imuOffsetInDegrees = -90; // Math.toDegrees(startPose.heading.toDouble());

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        // *** Route aligns with Decode_MeepMeep_Blue_ID22_Small_Triangle ***

        ReleaseGateFromFirstMark = drive.actionBuilder(new Pose2d(34.75, -55, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(0, -34),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(0,-58))
                .strafeToSplineHeading(new Vector2d(0, -48),Math.toRadians(-90))
                .build();

        ReleaseGateFromSecondMark = drive.actionBuilder(new Pose2d(11.5, -55, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(0, -34),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(0,-58))
                .strafeToSplineHeading(new Vector2d(0, -48),Math.toRadians(-90))
                .build();

        ReleaseGateFromThirdMark = drive.actionBuilder(new Pose2d(-11.75, -55, Math.toRadians(-90)))
                .strafeToSplineHeading(new Vector2d(0, -34),Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(0,-58))
                .strafeToSplineHeading(new Vector2d(0, -48),Math.toRadians(-90))
                .build();

        GateToSmallTriangle = drive.actionBuilder(new Pose2d(0, -48, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48,-12))
                .build();

        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48, -12))
                .build();

        DriveToLoadingZoneAndBack = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(48,-50),Math.toRadians(-25))
                .strafeToLinearHeading(new Vector2d(41,-59),Math.toRadians(-25))
                .strafeToLinearHeading(new Vector2d(60,-62),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(48,-12),Math.toRadians(-90))
                .build();


        DriveToThirdMark = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(-11.75, -30))
                .strafeToConstantHeading(new Vector2d(-11.75, -55))
                .build();

        DriveFirstMarkToSmallTriangle = drive.actionBuilder(new Pose2d(-11.75, -55, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48, -12))
                .build();

        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(11.5, -30))
                .strafeToConstantHeading(new Vector2d(11.5, -55))
                .build();

        DriveSecondMarkToSmallTriangle = drive.actionBuilder(new Pose2d(11.5, -55, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(20, -12))
                .strafeToConstantHeading(new Vector2d(48, -12))
                .build();

        DriveToFirstMark = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(34.75, -30))
                .strafeToConstantHeading(new Vector2d(34.75, -55))
                .build();

        DriveFirstMarkToSmallTriangle = drive.actionBuilder(new Pose2d(34.75, -55, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48, -12))
                .build();

        DriveOutOfLaunchZone = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(20, -12))
                .build();

        theRobot.SetAutoLifterMode(true);

        double obeliskID = theRobot.detectObeliskMotif(2000);

        // ***************************************************
        // ****  Secondary Thread to run all the time ********
        // ***************************************************
        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() && getRuntime() < 30) {
                theRobot.ShowTelemetry();
                //control.ShowPinpointTelemetry();

                theRobot.SetIndicatorLights();

                if (isStarted()) {
                    theRobot.RunAutoLifter();
                }
                telemetry.update();

                sleep(20);
            }
        });
        SecondaryThread.start();

        // turn turret to face the obelisk
        double turretTargetAngle = 91.0;
        theRobot.SetTurretRotationAngle(turretTargetAngle);
        sleep(1000);
        // turn off turret power so doesn't twitch
        theRobot.TurnOffTurret();
        theRobot.SetLaunchRampPosition(1.0);
        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        waitForStart();

        // ********* STARTED ********************************
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true,true);

        // set shooter speed to small triangle speed, can just leave at this speed the whole time
        double shooterSpeedRPM = 3200;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // Turn Turret towards target, can leave turret there the whole time
        turretTargetAngle = 115.0;
        theRobot.SetTurretRotationAngle(turretTargetAngle);

        // drive to the start triangle
        Actions.runBlocking(new SequentialAction(DriveToShootingPosition, setIntakeOff()));
        shooterSpeedRPM = 3400;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
        /* **** SHOOT BALL #1 **** */
        ShootBall(shooterSpeedRPM);

        // can stop the intake after the first shot to save power
        //control.SetIntakeMotor(false,true);

        /* **** SHOOT BALL #2 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #3 **** */
        ShootBall(shooterSpeedRPM);
        //drive to loading zone, get balls, and return to launch zone
        theRobot.SetIntakeMotor(true,true);
        Actions.runBlocking(
                new SequentialAction(
                DriveToLoadingZoneAndBack, setIntakeOff()
                ));
        // Shoot 3 balls

        /* **** SHOOT BALL #4 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #5 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #6 **** */
        ShootBall(shooterSpeedRPM);

        // Drive to middle line, get balls, and return to launch zone
        if (obeliskID == 21) {

            setIntakeOn();
            Actions.runBlocking(
                    new SequentialAction(
                            // Drive to the middle line and turn the intake on
                            DriveToFirstMark,
                            new SleepAction(1.0), // tiny sleep to finish ingesting balls, not sure how much is really needed
                            // Return to launch zone and turn intake off
                            setIntakeOff(),
                            ReleaseGateFromFirstMark,
                            GateToSmallTriangle
                    ));
        }
        else if (obeliskID == 22) {
            setIntakeOn();
            Actions.runBlocking(
                    new SequentialAction(
                            // Drive to the middle line and turn the intake on
                            DriveToSecondMark,
                            new SleepAction(1.0), // tiny sleep to finish ingesting balls, not sure how much is really needed
                            // Return to launch zone and turn intake off
                            setIntakeOff(),
                            ReleaseGateFromSecondMark,
                            GateToSmallTriangle
                    ));
        }
        else if (obeliskID == 23) {
            setIntakeOn();
            Actions.runBlocking(
                    new SequentialAction(
                            // Drive to the middle line and turn the intake on
                            DriveToThirdMark,
                            new SleepAction(1.0), // tiny sleep to finish ingesting balls, not sure how much is really needed
                            // Return to launch zone and turn intake off
                            setIntakeOff(),
                            ReleaseGateFromThirdMark,
                            GateToSmallTriangle
                    ));
        }

        // Shoot 3 balls

        /* **** SHOOT BALL #7 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #8 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #9 **** */
        ShootBall(shooterSpeedRPM);

        // store final exact position in blackboard, so can initialize absolute pinpoint with that position
        //control.pinpoint.update();
        drive.updatePoseEstimate();
        blackboard.put(Gericka_Hardware.FINAL_X_POSITION, drive.localizer.getPose().position.x);
        blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, drive.localizer.getPose().position.y);
        blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

        //lower lifter
        theRobot.SetLifterDown();
        theRobot.SetAutoLifterMode(false);
        
        // turn off shooter wheel
        shooterSpeedRPM = 0.0; //3200rpm was about the value observed when the Motor was commanded to 75%.
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // return turret to zero position
        turretTargetAngle = 0.0;
        theRobot.SetTurretRotationAngle(turretTargetAngle);

        Gericka_Hardware.autoTimeLeft = 30 - getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();

    }

    private void ShootBall(double shooterSpeedRPM) {
        double timeout = getRuntime() + 2.0; // only allow a max of 2 seconds delay for spin up waiting
        // sleep some time to allow shooter wheel to spin back up if needed

        // while neither motor is at least within 200 of our target speed
        while ((theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 200)) &&
                (theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 200)) &&
                (getRuntime() < timeout)) {
            theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM-100);
            sleep(20);
        }
        while ((theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100)) &&
                (theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100) ) &&
                (getRuntime() < timeout)) {
            theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM-50);
            sleep(20);
        }
        while ((theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 10) ||
                theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) > (shooterSpeedRPM + 200)) &&
                (theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 10) ||
                        theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) > (shooterSpeedRPM + 200)) &&
                (getRuntime() < timeout)) {
            theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            sleep(20);
        }
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        /* **** SHOOT a BALL  **** */
        theRobot.SetLifterUp();
        sleep(lifterUpSleepTime);

        // Reset to get another ball
        theRobot.SetLifterDown();
        sleep(lifterDownSleepTime);

    }

    public Action setIntakeOn() {
        return new setIntakeOn();
    }

    public class setIntakeOn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            theRobot.SetIntakeMotor(true, true);
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
            }
        }

        public Action setIntakeOff() {
            return new setIntakeOff();
        }

        public class setIntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }
                theRobot.SetIntakeMotor(false, false);
                packet.put("lock purple pixel", 0);
                return false;  // returning true means not done, and will be called again.  False means action is completely done
            }
        }

    public class SetLifterUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            theRobot.SetLifterUp();
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

    public class SetLifterDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            theRobot.SetLifterDown();
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

}
