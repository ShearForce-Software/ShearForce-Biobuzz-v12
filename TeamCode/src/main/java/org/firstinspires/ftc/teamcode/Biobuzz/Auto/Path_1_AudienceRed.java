package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "Path 1 Audience side Red", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class Path_1_AudienceRed extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        //Flower left plan 1
        // Target Poses
        Pose startPose = new Pose(25, 9, Math.toRadians(90));
        Pose Pose1 = new Pose(49, 24.5, Math.toRadians(75));
        Pose Pose2 = new Pose(10, 10, Math.toRadians(80));
        Pose Pose3 = new Pose(37, 115, Math.toRadians(310));
        Pose Pose4 = new Pose(14, 50, Math.toRadians(25));
        Pose Pose5 = new Pose(37,115, Math.toRadians(310));
        Pose Pose6 = new Pose(12,115, Math.toRadians(310));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToShoot1 = line(startPose, Pose1).linear(startPose, Pose1);
        Path driveToShoot2 = line(Pose1, Pose2).linear(Pose1, Pose2);
        Path driveToShoot3 = line(Pose2, Pose3).linear(Pose2, Pose3);
        Path driveToShoot4 = line(Pose3, Pose4).linear(Pose3, Pose4);
        Path driveToShoot5 = line(Pose4, Pose5).linear(Pose4, Pose5);
        Path driveToShoot6 = line(Pose5, Pose6).linear(Pose5, Pose6);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels
                        //shooter.startFlywheelsCommand(),

                        //Drive up to the hive and shoot preload
                        follow(follower, driveToShoot1),
                        parallel(
                                shooter.firePositionCommand(),
                                waitMs(1000)
                        ),


                        //Intake from the Garden
                        follow(follower, driveToShoot2),
                        parallel(
                                intake.turnOnCommand(),
                                waitMs(500)
                        ),
                        intake.turnOffCommand(),

                        //shoot on the other hive
                        follow(follower, driveToShoot3),
                        parallel(
                                shooter.firePositionCommand(),
                                waitMs(1000)
                        ),


                        //Drive and Intake from the flower
                        follow(follower, driveToShoot4),
                        parallel(
                                intake.turnOnCommand(),
                                waitMs(500)
                        ),
                        intake.turnOffCommand(),

                        //Drive up to the Hive and shoot!
                        follow(follower, driveToShoot5),
                        parallel(
                                shooter.firePositionCommand(),
                                waitMs(1000)
                        ),
                        //drive to the parking zone and park
                        follow(follower, driveToShoot6),
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
