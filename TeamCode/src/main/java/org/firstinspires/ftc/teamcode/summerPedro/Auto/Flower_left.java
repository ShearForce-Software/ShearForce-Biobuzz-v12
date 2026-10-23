package org.firstinspires.ftc.teamcode.summerPedro.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

@Autonomous(name = "Flower Left", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class Flower_left extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);

        ElapsedTime matchTimer = new ElapsedTime();
        //Flower left plan 1
        // Target Poses
        Pose startPose = new Pose(49, 128, Math.toRadians(0));
        Pose Pose1 = new Pose(15, 49, Math.toRadians(0));
        Pose Pose2 = new Pose(38, 110, Math.toRadians(0));
        Pose Pose3 = new Pose(10, 10, Math.toRadians(80));
        Pose controlPose = new Pose(28.46, 12.39, Math.toRadians(80));
        Pose Pose4 = new Pose(38, 23, Math.toRadians(80));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToShoot1 = line(startPose, Pose1).linear(startPose, Pose1);
        Path driveToShoot2 = line(Pose1, Pose2).linear(Pose1, Pose2);
        Path driveToShoot3   = line(Pose2, Pose3).linear(Pose2, Pose3);
        Path control = curve(Pose3, controlPose, Pose4).linear(Pose3, Pose4);
        Path driveToShoot4   = line(Pose3, Pose4).linear(Pose3, Pose4);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels
                        //shooter.startFlywheelsCommand(),
                        waitMs(2000), // waits 2000ms
                        // Segment 1: Drive to Shoot 1 & Shoot
                        follow(follower, driveToShoot1),
                        //shooter.firePositionCommand(),
                        waitMs(2000),
                        // Segment 2: Drive to Shoot 2 & Shoot
                        follow(follower, driveToShoot2),
                        //shooter.firePositionCommand(),
                        waitMs(1000),
                        // Segment 3: Park & stop flywheels
                        follow(follower, driveToShoot3),
                        //shooter.stopFlywheelsCommand(),

                        follow(follower, driveToShoot4),

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
