
package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// auto select manual opMode next
@Autonomous(name = "BLUE Far Shoot 3", preselectTeleOp = "Gericka 1 Manual Control")
public class Blue_Far_shoot_3 extends LinearOpMode {

    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    @Override
    public void runOpMode() {
        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "BLUE");

        // initialize roadrunner
        drive = new Gericka_MecanumDrive(hardwareMap, Gericka_Hardware.farBlueStartPose);
        theRobot.InitRoadRunner(drive);
        theRobot.buildCommonAutoRoutes();


        // initialize the webcam
        theRobot.WebcamInit(this.hardwareMap);

        // Turn turret toward the obelisk BEFORE scanning (useful if the camera is turret-mounted)
        //double turretTargetAngle = 116;
        theRobot.SetTurretRotationAngle(theRobot.BlueFarLaunchTurretAngle);
        theRobot.SetLaunchRampPosition(theRobot.FarLaunchHoodAngle);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // finish initializing pinpoint / roadrunner initial position
        sleep(500);
        theRobot.SetRoadrunnerInitialPosition(Gericka_Hardware.farBlueStartPose.position.x, Gericka_Hardware.farBlueStartPose.position.y, -90);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "BLUE");

        // set lifter half up (so can get 3 balls loaded in robot)
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);


        theRobot.SetAutoLifterMode(false);
        theRobot.SetShooterPIDF_Enabled(true);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;

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


        //double turretTargetAngleSmallTriangle = 114;
        theRobot.SetTurretRotationAngle(theRobot.BlueFarLaunchTurretAngle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true, true);

        // shooter speed for SMALL TRIANGLE
        //double shooterSpeedRPM = 3000;
        theRobot.SetShooterMotorToSpecificRPM(theRobot.FarLaunchRPM);

        // -------------------------
        // START -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
        // Drive to the shooting position
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(theRobot.BlueFarDriveStartingPositionToShootingPosition));
        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(false, true);
        theRobot.SetTurretRotationAngle(theRobot.BlueFarLaunchTurretAngle);

        // SHOOT-3
        sleep(12000);  // first time shooting give a tiny extra wait to allow shooter to spin up
        theRobot.SetIntakeMotor(true, true);
        theRobot.ShootAutoBalls();
        theRobot.SetIntakeMotor(false, true);
        drive.updatePoseEstimate();
        sleep(14000);

        // -------------------------
        // PARK
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(theRobot.BlueFarDriveTowardCenter, setIntakeOff()));

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
