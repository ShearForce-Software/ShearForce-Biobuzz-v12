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
        Pose startPose = new Pose(60, 135, Math.toRadians(90));
        Pose Pose1 = new Pose(49, 130, Math.toRadians(90));
        Pose controlPose = new Pose(61.32, 126.86, Math.toRadians(90));
        Pose Pose2 = new Pose(49, 119, Math.toRadians(110));
        Pose Pose3 = new Pose(14, 49, Math.toRadians(180));
        Pose controlPose2 = new Pose(26.7, 88.098, Math.toRadians(180));
        Pose Pose4 = new Pose(38, 110, Math.toRadians(140));
        Pose Pose5 = new Pose(10, 10, Math.toRadians(80));
        Pose controlPose3 = new Pose(11.46, 25, Math.toRadians(80));
        Pose Pose6 = new Pose(38, 25, Math.toRadians(-120));
        Pose Pose7 = new Pose(10, 102, Math.toRadians(180));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToShoot1 = line(startPose, Pose1).linear(startPose, Pose1);
        Path control = curve(Pose1, controlPose, Pose2).linear(Pose1, Pose2);
        Path driveToShoot2 = line(Pose2, Pose3).linear(Pose2, Pose3);
        Path driveToShoot3   = line(Pose3, Pose4).linear(Pose3, Pose4);
        Path driveToShoot4   = line(Pose4, Pose5).linear(Pose4, Pose5);
        Path control2 = curve(Pose5, controlPose3, Pose6).linear(Pose5, Pose6);
        Path driveToShoot5   = line(Pose6, Pose7).linear(Pose6, Pose7);


        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels
                        //shooter.startFlywheelsCommand(),
                        waitMs(1500), // waits 2000ms
                        // Segment 1: Drive to Shoot 1 & Shoot
                        follow(follower, driveToShoot1),
                        //shooter.firePositionCommand(),
                        waitMs(1000),
                        // Segment 2: Drive to Shoot 2 & Shoot
                        follow(follower, driveToShoot2),
                        //shooter.firePositionCommand(),
                        waitMs(1000),
                        // Segment 3: Park & stop flywheels
                        follow(follower, driveToShoot3),
                        //shooter.stopFlywheelsCommand(),
                        waitMs(2000),
                        follow(follower, driveToShoot4),
                        waitMs(1000),
                        follow(follower, driveToShoot5),
                        waitMs(1000),

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
