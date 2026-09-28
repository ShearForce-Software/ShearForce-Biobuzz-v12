package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// auto select manual opMode next
@Autonomous(name = "BLUE Close 9 BALL WITH GATE", preselectTeleOp = "Gericka 1 Manual Control")
public class

Blue_Close_9Ball_withGate extends LinearOpMode {

    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;

    @Override
    public void runOpMode() {
        //Pose2d startPose = new Pose2d(-60,-39,Math.toRadians(-90));
        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "BLUE");

        // initialize roadrunner
        drive = new Gericka_MecanumDrive(hardwareMap, Gericka_Hardware.closeBlueStartPose);
        theRobot.InitRoadRunner(drive);
        theRobot.buildCommonAutoRoutes();

        // initialize the webcam
        theRobot.WebcamInit(this.hardwareMap);

        // initialize the turret angle and launch ramp

        theRobot.SetTurretRotationAngle(theRobot.BlueCloseLaunchTurretAngle);
        theRobot.SetLaunchRampPosition(theRobot.CloseLaunchHoodAngle);
        // set lifter half up (so can get 3 balls loaded in robot)
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // finish initializing pinpoint / roadrunner initial position
        theRobot.SetRoadrunnerInitialPosition(Gericka_Hardware.closeBlueStartPose.position.x, Gericka_Hardware.closeBlueStartPose.position.y, Math.toRadians(-90));

        theRobot.SetAutoLifterMode(false);
        theRobot.SetShooterPIDF_Enabled(true);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "BLUE");

        // ***************************************************
        // ****  Secondary Thread to run all the time ********
        // ***************************************************
        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() && getRuntime() < 30) {
                theRobot.SetIndicatorLights();

                if (isStarted()) {
                    theRobot.RunAutoLifter();
                    theRobot.SetShooterPIDFCoefficients();
                }
                else {
                    theRobot.ShowTelemetry();
                    telemetry.update();
                }

                sleep(20);
            }
        });
        SecondaryThread.start();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        waitForStart();

        // ********* STARTED ********************************
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;

        // command the turret to power on
        theRobot.SetTurretRotationAngle(theRobot.BlueCloseLaunchTurretAngle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true, true);

        // spin up the shooter
        theRobot.SetShooterMotorToSpecificRPM(theRobot.CloseLaunchRPM);

        // -------------------------
        // START -> SHOOT FROM BIG TRIANGLE
        // -------------------------
        // Drive to the shooting position
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(theRobot.BlueCloseDriveCloseStartPositionToBigTriangle));

        // SHOOT-3
        // first time shooting give a tiny extra wait to allow shooter to finish spinning up
        sleep(theRobot.autoCloseSleepTime);        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);
        //theRobot.ShootAutoThreeBalls();
        theRobot.ShootAutoBalls();

        theRobot.SetIntakeMotor(false, true);

        drive.updatePoseEstimate();

        // -------------------------
        // BIG TRIANGLE -> THIRD STRIP -> WAIT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.BlueCloseDriveBigTriangleToThirdMark,
                        new SleepAction(0.250), // sleep time to finish intaking the balls
                        setIntakeOff(),
                        theRobot.BlueCloseDriveThirdMarkToLock
                )
        );
        // turn off intake to save battery power
        theRobot.SetIntakeMotor(false, true);

        //wait for Frooty Loops
        while ((getRuntime() < 13) && (!isStopRequested())) {
            sleep(20);
        }

        // -------------------------
        // THIRD STRIP -> GATE -> BIG TRIANGLE SHOOT
        // -------------------------
        Actions.runBlocking(
                new SequentialAction(

                        //new SleepAction(0.4), // sleep time to hold the gate open
                        new ParallelAction(theRobot.BlueCloseDriveLockToBigTriangle)
                )
        );

        drive.updatePoseEstimate();
        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);

        // SHOOT-3
        //theRobot.ShootAutoThreeBalls();
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);
        // -------------------------
        // BIG TRIANGLE -> SECOND MARK -> BIG TRIANGLE SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.BlueCloseDriveDriveBigTriangletoSecondMark,
                        new SleepAction(0.250),
                        setIntakeOff(),
                        theRobot.BlueCloseDriveDriveSecondMarktoBigTriangle
                )
        );
        drive.updatePoseEstimate();

        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);
        // SHOOT-3
        //theRobot.ShootAutoThreeBalls();
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);
        theRobot.SetTurretRotationAngle(0.0);

        // -------------------------
        // BIG TRIANGLE -> PARK NEXT TO GATE
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(theRobot.BlueCloseDriveDriveShootingPositionToGateLock, setIntakeOff()));

        // -------------------------
        // Cleanup
        // -------------------------
        drive.updatePoseEstimate();
        blackboard.put(Gericka_Hardware.FINAL_X_POSITION, drive.localizer.getPose().position.x);
        blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, drive.localizer.getPose().position.y);
        blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

        //lower lifter
        theRobot.SetLifterDown();
        theRobot.SetAutoLifterMode(false);

        // turn off shooter wheel
        theRobot.SetShooterMotorToSpecificRPM(0.0);

        // return turret to zero position
        theRobot.SetTurretRotationAngle(0.0);

        // turn off intake
        theRobot.SetIntakeMotor(false, true);

        Gericka_Hardware.autoTimeLeft = 30 - getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();

        while ((getRuntime() < 29.8) && (!isStopRequested())) {
            drive.updatePoseEstimate();
            blackboard.put(Gericka_Hardware.FINAL_X_POSITION, drive.localizer.getPose().position.x);
            blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, drive.localizer.getPose().position.y);
            blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            sleep(20);
        }
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
            return false;
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
            return false;
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
            return false;
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
            return false;
        }
    }
}
