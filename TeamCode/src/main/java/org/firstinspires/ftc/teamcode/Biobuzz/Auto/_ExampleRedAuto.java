package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Pedro 3 + Ivy Core Framework Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

// Static Ivy Command Structure Imports
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

/**
 * Example Autonomous OpMode showcasing PedroPathing and the Ivy command framework.
 *
 * Demonstrates:
 * - Setting up match configuration (Alliance color)
 * - Commands.conditional for dynamic camera-based branching at runtime
 * - Parallel execution: Running Intake & Transfer while driving along a path
 * - Concurrent firing: Firing the shooter while driving or standing
 * - Pose persistence across Autonomous and TeleOp transitions via MatchConfig
 */
@Autonomous(name = "ExampleRedAuto", group = "Testing", preselectTeleOp = "BIOBUZZ TeleOp")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class _ExampleRedAuto extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Record alliance color in MatchConfig
        MatchConfig.allianceColor = MatchConfig.AllianceColor.RED;

        // Instantiate Pedro Follower and Subsystems
        follower = Constants.create(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, intake);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Query camera control bounds and apply settings
        camera.queryCameraControlBoundsAndApply();

        // Start non-blocking background thread for color sensor updates
        transfer.startColorSensorThread();

        // Target Poses
        Pose startPose = new Pose(33, 9, Math.toRadians(180));
        Pose gardenPose = new Pose(11.0, 11.0, Math.toRadians(180));

        Pose audienceShootPose = new Pose(60, 24, Math.toRadians(180));
        Pose backstageShootPose = new Pose(36, 111, Math.toRadians(45));
        Pose flowerPose = new Pose(12, 48, Math.toRadians(180));
        Pose parkPose = new Pose(10, 10, Math.toRadians(0));

        // Set starting pose
        follower.setPose(startPose);

        // Define Path Segments
        Path driveToGarden = line(startPose, gardenPose).linear(startPose, gardenPose);

        Path driveToAudienceShoot1 = line(gardenPose, audienceShootPose).linear(gardenPose, audienceShootPose);

        Path driveToTVShoot1 = curve(audienceShootPose
                , new Pose(24, 48, Math.toRadians(0))
                , new Pose(24, 60, Math.toRadians(0))
                , new Pose(20, 96, Math.toRadians(0))
                , backstageShootPose).linear(Math.toRadians(180), Math.toRadians(0));

        Path driveToRedAudienceFlower = curve(audienceShootPose
                , new Pose(20, 90, Math.toRadians(0))
                , new Pose(24, 72, Math.toRadians(0))
                , new Pose(30, 60, Math.toRadians(0))
                , new Pose(20, 48, Math.toRadians(0))
                , flowerPose).linear(audienceShootPose, flowerPose);

        Path driveToAudienceShoot2 = line(flowerPose, audienceShootPose).linear(flowerPose, audienceShootPose);

        Path driveToTVShoot2 = curve(flowerPose
                , new Pose(24, 48, Math.toRadians(0))
                , new Pose(30, 60, Math.toRadians(0))
                , new Pose(20, 90, Math.toRadians(0))
                , backstageShootPose).linear(audienceShootPose, backstageShootPose);

        Path driveToPark = curve(backstageShootPose
                , new Pose(30, 96, Math.toRadians(0))
                , parkPose).linear(backstageShootPose, parkPose);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),

                        // 1. Shoot preloaded balls into audience side red hive using PIDF position control
                        transfer.shootAllBallsCommand(),

                        // 2. Drive to the red garden with intake on in parallel (transfer auto-advances via PIDF in main loop)
                        parallel(
                                follow(follower, driveToGarden),
                                intake.turnOnCommand()
                        ),

                        // 3. Drive to shoot position for audience side red hive, turning intake off
                        parallel(
                                follow(follower, driveToAudienceShoot1),
                                intake.turnOffCommand()
                        ),

                        // 4. Multi-frame camera check to verify Audience Goal cluster is visible AND upright (Goal UP)
                        instant(() -> MatchConfig.audienceGoalUp = camera.verifyAudienceGoalUpright()),
                        conditional(
                                MatchConfig::isAudienceGoalUp,
                                transfer.shootAllBallsCommand(),
                                sequential(
                                        follow(follower, driveToTVShoot1),
                                        transfer.shootAllBallsCommand()
                                )
                        ),

                        // 5. Drive to audience side red flower with intake on and de-pollinator down
                        parallel(
                                follow(follower, driveToRedAudienceFlower),
                                intake.turnOnCommand(),
                                intake.depollinatorDownCommand()
                        ),

                        // 6. Intake more pollen from flower
                        waitMs(1000),

                        // 7. Drive to shoot position for audience side red hive (turning intake off, and raising the de-pollinator)
                        parallel(
                                follow(follower, driveToAudienceShoot2),
                                intake.turnOffCommand(),
                                intake.depollinatorUpCommand()
                        ),

                        // 8. Multi-frame camera check to verify Audience Goal cluster is visible AND upright (Goal UP)
                        instant(() -> MatchConfig.audienceGoalUp = camera.verifyAudienceGoalUpright()),
                        conditional(
                                MatchConfig::isAudienceGoalUp,
                                transfer.shootAllBallsCommand(),
                                sequential(
                                        follow(follower, driveToTVShoot2),
                                        transfer.shootAllBallsCommand()
                                )
                        ),

                        // 9. Drive to parking spot and turn off flywheels & mechanisms
                        parallel(
                                shooter.stopFlywheelsCommand(),
                                intake.turnOffCommand(),
                                transfer.stopCommand(),
                                follow(follower, driveToPark)
                        ),

                        // Save final pose for TeleOp transition
                        instant(() -> MatchConfig.currentPose = follower.pose())
                )
        );

        waitForStart();
        matchTimer.reset();

        shooter.startFlywheels();
        shooter.setPollenPosition();

        // Execution loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            transfer.update();
            Scheduler.execute();
        }

        // Clean up background thread and camera on exit
        transfer.stopColorSensorThread();
        camera.close();
    }
}
