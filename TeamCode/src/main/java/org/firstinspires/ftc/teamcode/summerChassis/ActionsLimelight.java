package org.firstinspires.ftc.teamcode.summerChassis;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Limelight+Actions")
@Disabled
public class ActionsLimelight extends LinearOpMode{
    Pose2d startPose = new Pose2d(0,0,0);
    MecanumDrive_summerChassis drive = new MecanumDrive_summerChassis(hardwareMap, startPose);
    SummerChassis control = new SummerChassis(false, false, this);
    //control.Init(hardwareMap);




    public void runOpMode(){
        control.InitLimelight(hardwareMap);

        while(!isStarted()){
            telemetry.update();
        }

    }





}
