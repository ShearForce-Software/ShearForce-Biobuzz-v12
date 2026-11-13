package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

    @TeleOp(name = "Melody Centric Drive Test", group= "BIOBUZZ") // This flags the program as either Autonomous or TeleOp, affecting where it shows up on the Driver Station
    public class MelodyRobotCentricTest extends LinearOpMode { //You may need to add the "extends LinearOpMode" yourself

        @Override
        public void runOpMode() throws InterruptedException {

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

            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            waitForStart();

            while(opModeIsActive()) {

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

                // The next three sections set and update all telemetry
                telemetry.addData("Forward Power", ForwardPower);
                telemetry.addData("Strafe Power", StrafePower);
                telemetry.addData("Rotate Power", RotatePower);
                telemetry.addData("Vector Magnitude", VectorMagnitude);

                telemetry.addData("Back Left Velocity", ((DcMotorEx) BackLeft).getVelocity());
                telemetry.addData("Back Right Velocity", ((DcMotorEx) BackRight).getVelocity());
                telemetry.addData("Front Left Velocity", ((DcMotorEx) FrontLeft).getVelocity());
                telemetry.addData("Front Right Velocity", ((DcMotorEx) FrontRight).getVelocity());

                telemetry.update();
            }
        }
    }

