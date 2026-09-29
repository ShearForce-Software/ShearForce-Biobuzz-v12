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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Align", group="SummerChassis")
@Disabled
public class Align extends LinearOpMode{
    @Override
    public void runOpMode(){
        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive_summerChassis drive = new MecanumDrive_summerChassis(hardwareMap, startPose);
        SummerChassis control = new SummerChassis(false, false, this);
        //control.Init(hardwareMap);
        control.InitLimelight(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            double num = control.GetStrafeOffsetInInches("block");
            telemetry.addLine("current dist " + num);

            if (gamepad1.dpad_up && Math.abs(num)>0.2){
                Action strafeRightSlowAction = drive.actionBuilder(startPose)
                        .strafeToConstantHeading(
                                new Vector2d(0, num) // Strafe right by 10 inches
                                //slowStrafeVelocityConstraint, // Apply custom velocity constraint
                                //slowStrafeAccelConstraint // Apply custom acceleration constraint
                        )
                        .build();
                Actions.runBlocking(strafeRightSlowAction);
            }


            telemetry.update();

        }



    }

}
