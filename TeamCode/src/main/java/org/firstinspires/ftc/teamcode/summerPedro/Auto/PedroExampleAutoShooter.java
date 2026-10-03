package org.firstinspires.ftc.teamcode.summerPedro.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro 3 + Ivy Core Framework Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

// Static Ivy Command Structure Imports
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.instant;

import org.firstinspires.ftc.teamcode.summerPedro.pedro.Constants;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterSubsystem;

@Autonomous(name = "summerPedro Shooter", group = "summerPedro")
public class PedroExampleAutoShooter extends LinearOpMode {
    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private ElapsedTime matchTimer;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(0, 0, 0);
        Pose shootPose1 = new Pose(24, 24, Math.toRadians(45));
        Pose shootPose2 = new Pose(48, 12, Math.toRadians(0));
        Pose parkPose = new Pose(60, 0, Math.toRadians(90));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToShoot1 = line(startPose, shootPose1).linear(startPose, shootPose1);
        Path driveToShoot2 = line(shootPose1, shootPose2).linear(shootPose1, shootPose2);
        Path driveToPark   = line(shootPose2, parkPose).linear(shootPose2, parkPose);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels
                        shooter.startFlywheelsCommand(),

                        // Segment 1: Drive to Shoot 1 & Shoot
                        follow(follower, driveToShoot1),
                        shooter.firePositionCommand(),

                        // Segment 2: Drive to Shoot 2 & Shoot
                        follow(follower, driveToShoot2),
                        shooter.firePositionCommand(),

                        // Segment 3: Park & stop flywheels
                        follow(follower, driveToPark),
                        shooter.stopFlywheelsCommand(),

                        // Save final pose for TeleOp
                        instant(() -> PoseStorage.currentPose = follower.pose())
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
