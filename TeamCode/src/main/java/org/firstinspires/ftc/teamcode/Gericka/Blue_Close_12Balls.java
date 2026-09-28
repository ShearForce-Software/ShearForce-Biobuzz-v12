package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Blue Close Auto 12balls", preselectTeleOp ="Gericka 1 Manual Control")


public class Blue_Close_12Balls extends LinearOpMode {
    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;

    // Trajectories
    Action DriveStartToMidPosition;
    Action DriveMidToClosestLine;
    Action DriveClosestLineBackToLaunchPark;

    Action DriveClosestLineBackToMid;
    int lifterUpSleepTime = 300;
    int lifterDownSleepTime = 400;

    public void runOpMode(){
    //We will start at big triangle start
    //startPose = new Pose2d(-60,-39,Math.toRadians(-90));
    /* Initialize the Robot */
    theRobot.Init(hardwareMap, "BLUE");

    // initialize roadrunner
    drive = new Gericka_MecanumDrive(hardwareMap, Gericka_Hardware.closeBlueStartPose);
    theRobot.InitRoadRunner(drive);

    // initialize the webcam
    theRobot.WebcamInit(this.hardwareMap);

    sleep(500); // sleep at least 1/4 second to allow pinpoint to calibrate itself
    // finish initializing the pinpoint
    theRobot.SetRoadrunnerInitialPosition(Gericka_Hardware.closeBlueStartPose.position.x, Gericka_Hardware.closeBlueStartPose.position.y, Math.toRadians(-90));

    blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "BLUE");

    // set lifter half up (so can get 3 ball loaded in robot)
    theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);


    DriveStartToMidPosition = drive.actionBuilder(new Pose2d(theRobot.closeBlueStartPose.position.x, theRobot.closeBlueStartPose.position.y, Math.toRadians(-90)))
            .strafeToConstantHeading(new Vector2d(-11.4, -10))
            .build();

    // Mid (-10, -10, -90) -> closest line (-10, -40)
    DriveMidToClosestLine = drive.actionBuilder(new Pose2d(-11.4, -10, Math.toRadians(-90)))
            .strafeToConstantHeading(new Vector2d(-11.4, -55))
            .build();

    DriveClosestLineBackToMid = drive.actionBuilder(new Pose2d(-11.4, -55, Math.toRadians(-90)))
            .strafeToConstantHeading(new Vector2d(-11.4, -10))
            .build();

    // Closest line (-10, -40, -90) -> launch park (-54, -16)
    DriveClosestLineBackToLaunchPark = drive.actionBuilder(new Pose2d(-11.4, -10, Math.toRadians(-90)))
            .strafeToConstantHeading(new Vector2d(-54, -16))
            .build();

        theRobot.SetAutoLifterMode(false);

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

    // Turret initial rough angle toward speaker (tune as needed)
    double turretTargetAngle = 142

            ;
    theRobot.SetTurretRotationAngle(theRobot.BlueCloseLaunchTurretAngle);

    // turn off turret power so doesn't twitch
    //theRobot.turretMotor.setPower(0);

    // *************************************
    //      Wait for start
    // *************************************
        waitForStart();

    // ********* STARTED ********************************
    resetRuntime();
    Gericka_Hardware.autoTimeLeft = 0.0;


    theRobot.SetIntakeMotor(true,true);
    // spin up shooter wheel to max
    //theRobot.SetShooterSpeed(1.0);
    theRobot.SetShooterMotorToSpecificRPM(theRobot.CloseLaunchRPM);

    Actions.runBlocking(new SleepAction((1)));
    Actions.runBlocking(DriveStartToMidPosition);

    theRobot.SetIntakeMotor(false,true);

    // Turn turret more directly to target for auto shooting (tune on field)
    //turretTargetAngle = 45;    // CHANGE LATER
    //control.SetTurretRotationAngle(turretTargetAngle);

    // Shooter RPM for big triangle shots (tune as needed)
    double shooterSpeedRPM = 2700;
    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);


        /* **** SHOOT BALL #1 **** */
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true,true);
        /* **** SHOOT BALL #2 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #3 **** */
        ShootBall(shooterSpeedRPM);

        //Should we turn intake on while we go to the closest line
        theRobot.SetIntakeMotor(true,true);



    Actions.runBlocking(
            new SequentialAction(
                    // Mid -> closest line
                    DriveMidToClosestLine,
                              // tiny sleep to finish ingesting rings
                    // Closest line -> launch park

                    DriveClosestLineBackToMid

                    )
    );
        theRobot.SetIntakeMotor(false,true);

        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true,true);
        ShootBall(shooterSpeedRPM);
        ShootBall(shooterSpeedRPM);

        //lower lifter
        theRobot.SetLifterDown();
        theRobot.SetAutoLifterMode(false);

        // return turret to zero position
        //turretTargetAngle = 0.0;
        theRobot.SetTurretRotationAngle(0);
        //sleep(5000);

        // turn off shooter wheel
        shooterSpeedRPM = 0.0; //3200rpm was about the value observed when the Motor was commanded to 75%.
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // turn off intake motor
        theRobot.SetIntakeMotor(false,false);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        DriveClosestLineBackToLaunchPark
                )
        );

        // store final exact position in blackboard, so can initialize absolute pinpoint with that position
        //control.pinpoint.update();
        drive.updatePoseEstimate();
        blackboard.put(Gericka_Hardware.FINAL_X_POSITION, drive.localizer.getPose().position.x);
        blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, drive.localizer.getPose().position.y);
        blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));


       Gericka_Hardware.autoTimeLeft = 30-getRuntime();
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

    private void ShootBall(double shooterSpeedRPM) {
        // sleep some time to allow shooter wheel to spin back up if needed
        while (theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 10) ||
                theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) > (shooterSpeedRPM + 10)) {
            sleep(20);
        }

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
