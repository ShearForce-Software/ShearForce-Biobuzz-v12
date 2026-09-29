package org.firstinspires.ftc.teamcode.summerChassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;

@Autonomous(name="Strafe Right 10 Inches Slow", group="SummerChassis")
@Disabled
public class StrafeRightAutoRoutine extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1) Initialize the robot's drive system
        Pose2d startPose = new Pose2d(0, 0, 0); // Starting at origin
        MecanumDrive_summerChassis drive = new MecanumDrive_summerChassis(hardwareMap, startPose);

        // 2) Define custom constraints for slow strafing
        //double slowMaxVel = 15.0; // inches per second
        //double slowMaxAccel = 10.0; // inches per second squared

        //VelConstraint slowStrafeVelocityConstraint = new TranslationalVelConstraint(slowMaxVel);
        //AccelConstraint slowStrafeAccelConstraint = new ProfileAccelConstraint(-slowMaxAccel, slowMaxAccel);

        // 3) Build the slow strafe trajectory using custom constraints
        Action strafeRightSlowAction = drive.actionBuilder(startPose)
                .strafeToConstantHeading(
                        new Vector2d(0, -6) // Strafe right by 10 inches
                        //slowStrafeVelocityConstraint, // Apply custom velocity constraint
                        //slowStrafeAccelConstraint // Apply custom acceleration constraint
                )
                .build();


        // 4) Wait for the game to start
        telemetry.addLine("Ready to strafe right slowly...");
        telemetry.update();
        waitForStart();

        // 5) Execute the slow strafe action
        if (opModeIsActive()) {
            Actions.runBlocking(strafeRightSlowAction);
            telemetry.addLine("Slow strafe right action complete!");
            telemetry.update();
        }
    }
}
