package org.firstinspires.ftc.teamcode.summerPedro.Auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.summerPedro.pedro.Constants;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterSubsystem;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "PedoPathing Test", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class PedroPath_test extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        //IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
       // ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(40, 10, Math.toRadians(90));
        Pose shootPose1 = new Pose(40, 34, Math.toRadians(90));
        Pose shootPose2 = new Pose(64, 34, Math.toRadians(90));
        Pose shootPose3 = new Pose(64, 10, Math.toRadians(90));
        Pose parkPose = new Pose(40, 10, Math.toRadians(90));



        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path forward = line(startPose, shootPose1).linear(startPose, shootPose1);
        Path right = line(shootPose1, shootPose2).linear(shootPose1, shootPose2);
        Path back   = line(shootPose2, shootPose3).linear(shootPose2, shootPose3);
        Path left = line(shootPose3, parkPose).linear(shootPose3, parkPose);




        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels
                        //shooter.startFlywheelsCommand(),

                        // Segment 1: Drive to Shoot 1 & Shoot
                        follow(follower, forward),
                        //shooter.firePositionCommand(),

                        // Segment 2: Drive to Shoot 2 & Shoot
                        follow(follower, right),
                        //shooter.firePositionCommand(),

                        // Segment 3: Park & stop flywheels
                        follow(follower, back),
                        //shooter.stopFlywheelsCommand(),

                        follow(follower, left),

                        // Save final pose for TeleOp
                        instant(() -> MatchConfig.currentPose = follower.pose())
                )
        );

        waitForStart();
        matchTimer.reset();

        // Execution loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Scheduler.execute();
        }
    }
}
