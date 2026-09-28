package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// auto select manual opMode next
    @Autonomous(name = "RED FAR CLEANUP V3", preselectTeleOp = "Gericka 1 Manual Control")
public class

Red_Far_Cleanup_V3 extends LinearOpMode {

    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;

    @Override
    public void runOpMode() {
        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "RED");
        drive = new Gericka_MecanumDrive(hardwareMap, Gericka_Hardware.farRedStartPose);
        theRobot.InitRoadRunner(drive);
        theRobot.buildCommonAutoRoutes();
        theRobot.WebcamInit(this.hardwareMap);
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);
        theRobot.SetAutoLifterMode(false);
        theRobot.SetShooterPIDF_Enabled(true);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;

        // Turn turret toward the target
        theRobot.SetTurretRotationAngle(theRobot.RedFarLaunchTurretAngle);
        theRobot.SetLaunchRampPosition(theRobot.FarLaunchHoodAngle);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // finish initializing pinpoint / roadrunner initial position
        sleep(1500);
        theRobot.SetRoadrunnerInitialPosition(Gericka_Hardware.farRedStartPose.position.x, Gericka_Hardware.farRedStartPose.position.y, Gericka_Hardware.farRedStartPose.heading.toDouble());

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "RED");
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

        // re-command the turret angle to get the power back on
        theRobot.SetTurretRotationAngle(theRobot.RedFarLaunchTurretAngle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true, true);

        // shooter speed for SMALL TRIANGLE
        //double shooterSpeedRPM = SMALL_TRIANGLE_RPM;
        theRobot.SetShooterMotorToSpecificRPM(theRobot.FarLaunchRPM);

        // -------------------------
        // START -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
        // Drive to the shooting position
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(theRobot.RedFarDriveFarStartPositionToShootingPosition));
        drive.updatePoseEstimate();

        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(false, true);

        // SHOOT-3
        sleep(theRobot.autoSleepTime);  // first time shooting give a tiny extra wait to allow shooter to spin up
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);


                // -------------------------
                // SMALL TRIANGLE -> FIRST STRIP -> SMALL TRIANGLE -> SHOOT
                // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.RedFarDriveShootingPositionToFirstMark,
                        setIntakeOff(),
                        //new SleepAction(0.250),
                        theRobot.RedFarCleanUpDriveFirstMarkToShootingPosition
                )
        );
        drive.updatePoseEstimate();

        // SHOOT-3
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);

        // -------------------------
        // SMALL TRIANGLE -> COLLECT BALLS FROM GATE -> SMALL TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.RedFarCleanUpDriveShootingPositionToCollectGateBalls,
                        //new SleepAction(0.250),
                        setIntakeOff(),
                        theRobot.RedFarCleanUpDriveCollectGateBallsToShootingPosition
                )
        );
        drive.updatePoseEstimate();

        // SHOOT-3
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);

        // -------------------------
        // SMALL TRIANGLE -> COLLECT BALLS FROM GATE -> SMALL TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.RedFarCleanUpDriveShootingPositionToCollectGateBalls2,
                        //new SleepAction(0.250),
                        setIntakeOff(),
                        theRobot.RedFarCleanUpDriveCollectGateBallsToShootingPosition2
                )
        );
        drive.updatePoseEstimate();

        // SHOOT-3
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);


        // Command Turret to start storing itself at 0
        theRobot.SetTurretRotationAngle(0.0);

        // -------------------------
        // SMALL TRIANGLE -> PARK
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(theRobot.RedFarDriveOutOfSmallTriangle, setIntakeOff()));

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
