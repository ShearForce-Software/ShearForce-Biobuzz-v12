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
@Autonomous(name = "RED FAR 12 BALL WITH GATE", preselectTeleOp = "Gericka 1 Manual Control")
public class

Red_Far_12Ball_withGate extends LinearOpMode {

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
        //double turretTargetAngle = SMALL_TRIANGLE_TARGET_ANGLE;
        theRobot.SetTurretRotationAngle(theRobot.RedFarLaunchTurretAngle);
        theRobot.SetLaunchRampPosition(theRobot.FarLaunchHoodAngle);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // finish initializing pinpoint / roadrunner initial position
        sleep(500);
        theRobot.SetRoadrunnerInitialPosition(Gericka_Hardware.farRedStartPose.position.x, Gericka_Hardware.farRedStartPose.position.y, theRobot.farRedStartPose.heading.toDouble());

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
        Actions.runBlocking(theRobot.RedFarDriveFarStartPositionToShootingPosition);
        drive.updatePoseEstimate();

        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(false, true);

        // SHOOT-3
        sleep(theRobot.autoSleepTime);  // first time shooting give a tiny extra wait to allow shooter to spin up
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);

        // Change turret and shooter speeds for Big Triangle shots
        theRobot.SetLaunchRampPosition(theRobot.CloseLaunchHoodAngle);
        //shooterSpeedRPM = BIG_TRIANGLE_RPM;
        theRobot.SetShooterMotorToSpecificRPM(theRobot.CloseLaunchRPM);
        //turretTargetAngle = BIG_TRIANGLE_TARGET_ANGLE;
        theRobot.SetTurretRotationAngle(theRobot.RedCloseLaunchTurretAngle);

        // -------------------------
        // -> SECOND STRIP -> OPEN-GATE -> BIG TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.RedFarDriveShootingPositionToSecondMark,
                        //new SleepAction(0.250),
                        setIntakeOff(),
                        theRobot.RedFarDriveSecondMarkToLock,
                        new SleepAction(1.5),
                        theRobot.RedFarDriveLockToBigTriangle
                )
        );

        drive.updatePoseEstimate();

        // SHOOT-3
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);

        // -------------------------
        // BIG TRIANGLE -> THIRD STRIP -> BIG TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.RedFarDriveBigTriangleToThirdMark,
                        //new SleepAction(0.250),
                        setIntakeOff(),
                        theRobot.RedFarDriveThirdMarkToBigTriangle
                )
        );
        drive.updatePoseEstimate();

        // SHOOT-3
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);

        // set turret and shooting wheel to small triangle shots
        theRobot.SetLaunchRampPosition(theRobot.FarLaunchHoodAngle);
        theRobot.SetShooterMotorToSpecificRPM(theRobot.FarLaunchRPM);
        theRobot.SetTurretRotationAngle(theRobot.RedFarLaunchTurretAngle);

        // -------------------------
        // BIG TRIANGLE -> FIRST STRIP -> SMALL TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        setIntakeOn(),
                        theRobot.RedFarDriveBigTriangleToFirstMark,
                        //new SleepAction(0.250),
                        setIntakeOff(),
                        theRobot.RedFarDriveFirstMarkToShootingPosition
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
        // SMALL TRIANGLE -> PARK NEXT TO GATE
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(theRobot.RedFarDriveOutOfShootingPosition);

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
