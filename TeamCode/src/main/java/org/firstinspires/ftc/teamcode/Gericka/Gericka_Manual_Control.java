package org.firstinspires.ftc.teamcode.Gericka;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.util.Objects;

@TeleOp(name = "Gericka 1 Manual Control")
//@Disabled
public class Gericka_Manual_Control extends LinearOpMode {
    Gericka_Hardware theRobot;
    //public int turretTrackingID = 24; // default to Red
    final float TURRET_ROTATION_ANGLE_INCREMENT = 15.0f;
    final double shooterSpeedRPMIncrement = 50;
    boolean resetTurretEnabled = true;

    public void runOpMode() {
        theRobot = new Gericka_Hardware(true, true, this);
        telemetry.setMsTransmissionInterval(11);

        // Use the Blackboard to determine what color we should be
        Object allianceColor = blackboard.get(Gericka_Hardware.ALLIANCE_KEY);
        double defaultHeadingDegrees = 0.0;
        if (Objects.equals(allianceColor, "RED")) {
            theRobot.Init(this.hardwareMap, "RED");
            theRobot.SetAprilTagTargetId(24);
            defaultHeadingDegrees = 90.0;
        } else {
            theRobot.Init(this.hardwareMap, "BLUE");
            theRobot.SetAprilTagTargetId(20);
            defaultHeadingDegrees = -90.0;
        }

        theRobot.WebcamInit(this.hardwareMap);

        sleep(500); // sleep at least 1/4 second to allow pinpoint to calibrate itself

        // Use the Blackboard to determine the initial x,y and heading that auto finished in, OR use camera view of April tag to set position
        double xPositionInches = 0.0;
        double yPositionInches = 0.0;
        double headingDegrees = defaultHeadingDegrees;
        try {
            xPositionInches = (double) blackboard.get(Gericka_Hardware.FINAL_X_POSITION);
            yPositionInches = (double) blackboard.get(Gericka_Hardware.FINAL_Y_POSITION);
            headingDegrees = (double) blackboard.get(Gericka_Hardware.FINAL_HEADING_DEGREES);
            theRobot.SetRoadrunnerInitialPosition(xPositionInches, yPositionInches, headingDegrees);
        } catch (NullPointerException ignored) {
            //theRobot.SetPinpointPosition(0.0, 0.0, defaultHeadingDegrees);
            //theRobot.pinpoint.update();
            // attempt to use any april tags that are visible to update the pinpoint position
            theRobot.SetRoadrunnerPositionFromWebcam();
        }

        // set auto settings

        theRobot.SetAutoShooterMode(true);  // auto adjusts speed of shooter motors based on distance
        theRobot.SetUseOnlyWebcamForDistance(false);  // true means only use webcam, false means use pinpoint and webcam for distance calculations of shooter speed
        theRobot.SetAutoLifterMode(false);   // auto lifts lifter half-way if ball detected on top of lifter arm
        theRobot.SetTurretAutoMode(true);   // auto adjusts the turret rotation angle to align with the target april tag
        theRobot.SetUseRoadrunnerForTurretAnglesEnabled(true); // if true then will use pinpoint position to calculate turret angles if webcam target not visible
        theRobot.SetAutoIntakeMode(true);   // auto intakes balls when sensors detect room for another ball and ball present, auto turns off intake when full or nothing present
        theRobot.SetShooterPIDF_Enabled(true);
        //theRobot.SetTurretPIDF_Enabled(false); //TODO enable this when PIDF for turret is tuned
        theRobot.SetUpdateRoadrunnerFromWebcamEnabled(false);
        theRobot.SetAutoHoodMode(true);
        theRobot.SetTurretAutoResetMode(true);

        Pose2d startPose = new Pose2d(xPositionInches, yPositionInches, Math.toRadians(headingDegrees));
        Gericka_MecanumDrive drive = new Gericka_MecanumDrive(hardwareMap, startPose);
        theRobot.InitRoadRunner(drive);

        //theRobot.TestLights();
        //theRobot.light1Color();

        theRobot.ShowTelemetry();
        // ***************************************************
        // ****  Secondary Thread to run all the time ********
        // ***************************************************
        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() ) {
                //control.ShowPinpointTelemetry();

                theRobot.SetIndicatorLights();

                if (isStarted()) {

                    theRobot.SetShooterPIDFCoefficients(); // does nothing unless shooterPIDF_Enabled and PIDF values have been changed
                    theRobot.SetTurretPIDFCoefficients();
                    // Run the Auto Lift to Midway position (if enabled)
                    theRobot.RunAutoLifter();

                    theRobot.RunAutoIntake();  // this method may sleep for a little while, so needs to be in a separate thread from main

                    if (theRobot.GetTurretAutoMode()) {
                        theRobot.adjustTurretToTargetAprilTag();
                    }

                }

                theRobot.ShowTelemetry();

                sleep(5);
            }
        });
        SecondaryThread.start();

        waitForStart();
        resetRuntime();
        theRobot.setKickstandsUp();
        theRobot.SetLifterDown();// setting shoot lifter down at the start.


        boolean gamepad1_optionsWasPressed = false;
        boolean gamepad1_shareWasPressed = false;
        boolean gamepad2_optionsWasPressed = false;
        boolean gamepad2_shareWasPressed = false;

        while (opModeIsActive()) {
            theRobot.EndgameBuzzer();
            theRobot.drive.updatePoseEstimate();
            /*
            small triangle
            velocity:1620.000 L, 1600.000 R
            power: 0.690 L, 0.690 R

            close position
            velocity:
            power:

            Current PIDF values
            PIDF: 10p, 3i, 0d, 0f

            NEW Values
            PIDF: 0p, 2i, 70d, 15f

            small triangle
            PIDF: 7.5p 0i, 0d, 13.25f

            big triangle
            PIDF: 7.5p 0i, 0d, 14f

            fruityloops 40"
            PIDF: 7.5p 0i, 0d, 14.25f

            24inch
            PIDF: 7.5p 0i, 0d, 13f
             */


            gamepad1_optionsWasPressed = gamepad1.optionsWasPressed();
            gamepad1_shareWasPressed = gamepad1.shareWasPressed();
            gamepad2_optionsWasPressed = gamepad2.optionsWasPressed();
            gamepad2_shareWasPressed = gamepad2.shareWasPressed();

            //theRobot.detectObeliskMotif();

            /* *************************************************
             *************************************************
             * Driver Controls (gamepad1)
             *************************************************
             *************************************************
             */
            // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
            theRobot.RunDriveControls();
            // RESERVED COMBOS    options + cross and options + circle

            // Press the triangle button / "y" while facing directly away from the driver to set the IMU correctly for field-centric mode if off
            if (gamepad1.triangleWasPressed()) {
                if (gamepad1_optionsWasPressed || gamepad1.options) {
                    theRobot.SetFieldCentricMode(true);
                } else if (gamepad1_shareWasPressed || gamepad1.share){
                    theRobot.resetTurretAndPosition();
                }
                else {
                    theRobot.imu.resetYaw();
                }
            }
            else if (gamepad1.dpadUpWasPressed()) {
                if (gamepad1_optionsWasPressed || gamepad1.options) {
                    theRobot.resetPositionOnly();
                }
            }
            // robot centric drive mode
            else if (gamepad1.squareWasPressed()) {
                if (gamepad1_optionsWasPressed || gamepad1.options) {
                    theRobot.SetFieldCentricMode(false);
                }
            }
            else if (gamepad1.dpadLeftWasPressed())
            {
                if (gamepad1_optionsWasPressed || gamepad1.options) {
                    theRobot.SetTurretAutoMode(false);
                    theRobot.SetTurretRotationAngle(-90.0);
                }
            }
            else if (gamepad1.dpadRightWasPressed())
            {
                if (gamepad1_optionsWasPressed || gamepad1.options) {
                    theRobot.SetTurretAutoMode(false);
                    theRobot.SetTurretRotationAngle(90.0);
                }
            }
            else if (gamepad1.dpadDownWasPressed())
            {
                if (gamepad1_optionsWasPressed || gamepad1.options) {
                    theRobot.SetTurretAutoMode(false);
                    theRobot.SetTurretRotationAngle(0.0);
                }
            }


            // ********  KICKSTAND CONTROLS ***********************
            if (!gamepad1_optionsWasPressed || (!gamepad1.options) ||!gamepad2_optionsWasPressed || (!gamepad2.options)) {
                if (gamepad1.circleWasPressed()) {
                    theRobot.setKickstandsUp();
                    theRobot.SetAutoLifterMode(true);
                } else if (gamepad1.crossWasPressed()) {
                    theRobot.setKickstandsDown();
                    // put the lifter down too
                    theRobot.SetAutoLifterMode(false);
                    theRobot.SetLifterDown();
                    theRobot.SetTurretAutoMode(false);
                    theRobot.SetTurretRotationAngle(0);
                    theRobot.SetAutoShooterMode(false);
                    theRobot.SetShooterMotorToSpecificRPM(0);
                    theRobot.SetAutoIntakeMode(false);
                    theRobot.SetIntakeMotor(false, false);
                    theRobot.SetTurretAutoResetMode(false);


                }
            }
            /* *************************************************
             *************************************************
             * Arm Controls (gamepad2)
             *
             ***************************************              **********
             *************************************************
             */

            // ********   INTAKE & Turret MOTOR CONTROLS ***********************
            if (gamepad2.triangleWasPressed()) {
                if (gamepad2_optionsWasPressed || gamepad2.share || gamepad2_shareWasPressed || gamepad2.options) {
                    if (theRobot.GetAprilTagTargetId() == 20) {
                        // switch turret tracking to RED target
                        //turretTrackingID = 24;
                        theRobot.SetAprilTagTargetId(24);
                        theRobot.SetTurretAutoMode(true);
                        theRobot.SetAllianceColor("RED");
                        //theRobot.light1.setPosition(0.279);
                    } else {
                        // switch turret tracking to BLUE target
                        //theRobot.light1.setPosition(0.611);
                        theRobot.SetAllianceColor("BLUE");
                        //turretTrackingID = 20;
                        theRobot.SetAprilTagTargetId(20);
                        theRobot.SetTurretAutoMode(true);
                    }
                } else {
                    //Turn intake off
                    //theRobot.SetIntakeMotor(true,true);
                    theRobot.SetIntakeMotor(false, false);
                    theRobot.SetAutoIntakeMode(false);
                }
            } else if (gamepad2.circleWasPressed() && !gamepad2_optionsWasPressed) {
                if (gamepad2_shareWasPressed || gamepad2.share) {
                    //Turn Turret clockwise
                    theRobot.SetTurretRotationAngle(theRobot.getTurretTargetAngle() + TURRET_ROTATION_ANGLE_INCREMENT);
                    theRobot.SetTurretAutoMode(false);
                } else {
                    //Turn intake on
                    theRobot.SetIntakeMotor(true, true);
                    theRobot.SetAutoIntakeMode(false);
                }
            } else if (gamepad2.squareWasPressed()) {
                if (gamepad2_shareWasPressed || gamepad2.share) {
                    //Turn Turret counterclockwise
                    theRobot.SetTurretRotationAngle(theRobot.getTurretTargetAngle() - TURRET_ROTATION_ANGLE_INCREMENT);
                    theRobot.SetTurretAutoMode(false);
                }else if (gamepad2_optionsWasPressed || gamepad2.options) {
                    theRobot.resetTurret();
                } else {
                    //Turn outtake system on
                    theRobot.SetIntakeMotor(true, false);
                    theRobot.SetAutoIntakeMode(false);
                }
            } else if (gamepad2.right_stick_x > 0.1) {
                theRobot.SetTurretRotationAngle(theRobot.getTurretTargetAngle() + TURRET_ROTATION_ANGLE_INCREMENT);
                theRobot.SetTurretAutoMode(false);
            } else if (gamepad2.right_stick_x < -0.1) {
                theRobot.SetTurretRotationAngle(theRobot.getTurretTargetAngle() - TURRET_ROTATION_ANGLE_INCREMENT);
                theRobot.SetTurretAutoMode(false);
            } else if (gamepad2.crossWasPressed() && !gamepad2_optionsWasPressed) {
                if (gamepad2_shareWasPressed || gamepad2.share) {
                    // toggling auto intake on/off
                    theRobot.SetAutoIntakeMode(!theRobot.GetAutoIntakeMode());
                } else {
                    // toggling turret goal auto centering/tracking on/off
                    theRobot.SetTurretAutoMode(!theRobot.GetTurretAutoMode());
                }
            }

            // ********   SHOOTER MOTOR CONTROLS ***********************
            else if (gamepad2.dpadUpWasPressed()) {
                if (gamepad2_optionsWasPressed || gamepad2.options)
                {
                    theRobot.SetTurretRotationAngle(0.0);
                    theRobot.SetTurretAutoMode(false);
                }
                else {
                theRobot.SetAutoShooterMode(false);
                theRobot.SetShooterMotorToSpecificRPM(0.0);
                }
            } else if (gamepad2.dpadLeftWasPressed()) {
                if (gamepad2_optionsWasPressed || gamepad2.options) {
                    theRobot.SetAutoShooterMode(false);
                    theRobot.SetShooterMotorToSpecificRPM(2300);
                } else if (gamepad2_shareWasPressed || gamepad2.share){
                    theRobot.SetAutoShooterMode(false);
                    theRobot.SetShooterMotorToSpecificRPM(theRobot.GetShooterTargetRPM() - shooterSpeedRPMIncrement);
                }
            } else if (gamepad2.dpadRightWasPressed()) {
                if (gamepad2_optionsWasPressed || gamepad2.options) {
                    theRobot.SetAutoShooterMode(false);
                    theRobot.SetShooterMotorToSpecificRPM(2900);
                }
                else if (gamepad2_shareWasPressed || gamepad2.share) {
                    theRobot.SetAutoShooterMode(false);
                    theRobot.SetShooterMotorToSpecificRPM(theRobot.GetShooterTargetRPM() + shooterSpeedRPMIncrement);
                } else {
                    theRobot.SetAutoHoodMode(!theRobot.GetAutoHoodMode());
                }
            } else if (gamepad2.dpadDownWasPressed()) {
                if (gamepad2_optionsWasPressed || gamepad2.options) {
                    theRobot.SetAutoShooterMode(false);
                    theRobot.SetShooterMotorToSpecificRPM(3550);
                }
                else if (gamepad2_shareWasPressed || gamepad2.share){
                    theRobot.SetAutoHoodMode(!theRobot.GetAutoHoodMode());
                }
                else {
                    theRobot.SetAutoShooterMode(!theRobot.GetAutoShooterMode());
                }
            }
            // ********   Manual LAUNCH RAMP CONTROLS ***********************
            else if (gamepad2.left_stick_y > 0.1){
                theRobot.SetLaunchRampPosition(theRobot.GetLaunchRampPosition() - 0.1);
                theRobot.SetAutoHoodMode(false);
            }
            else if (gamepad2.left_stick_y < -0.1) {
                theRobot.SetLaunchRampPosition(theRobot.GetLaunchRampPosition() + 0.1);
                theRobot.SetAutoHoodMode(false);
            }

            // ********   LIFTER CONTROLS ***********************
            else if (gamepad2.rightBumperWasPressed()) {
                //Set lifter position to up
                theRobot.SetLifterUp();
            } else if (gamepad2.rightBumperWasReleased()) {
                theRobot.SetLifterDown();
            //} else if (gamepad1.rightBumperWasPressed()) {
                //Set lifter position to up
            //    theRobot.SetLifterUp();
            //} else if (gamepad1.rightBumperWasReleased()) {
            //    theRobot.setLifterDown();
            } else if (gamepad2.leftBumperWasPressed()) {
                theRobot.GamePad2LeftBumper = true;
                theRobot.SetLifterUp();
            }
            else if (gamepad2.leftBumperWasReleased()){
                theRobot.GamePad2LeftBumper = false;
                theRobot.SetLifterDown();
            }
            else if (gamepad2.left_trigger > 0.2) {
                //Set lifter position to middle
                theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);
            } else if (gamepad2.right_trigger > 0.2) {
                if (gamepad2_shareWasPressed || gamepad2.share) {
                    theRobot.SetAutoLifterMode(!theRobot.GetAutoLifterMode());
                }
            }

            theRobot.CalculateDistanceToTarget(); // always call this, even if not using it so that we can get the telemetry data
            if (theRobot.GetAutoShooterMode()) {
                if (theRobot.GetUseOnlyWebcamForDistance()) {
                    theRobot.SetShooterRPMFromWebCam();
                } else {
                    theRobot.SetShooterRPMFromRoadrunner();
                }
            }

            /*if (theRobot.GetTurretAutoMode()) {
                theRobot.adjustTurretToTargetAprilTag();
            }*/

            if (theRobot.GetUpdateRoadrunnerFromWebcamEnabled()) {
                // update the roadrunner position based on Webcam detected april tag if enabled
                theRobot.SetRoadrunnerPositionFromWebcam();
            }
            theRobot.checkTurretLimit();

            if (theRobot.GetTurretAutoResetMode()){
                theRobot.autoTurretReset();
            }

            /*
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


            //theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

    }
}

