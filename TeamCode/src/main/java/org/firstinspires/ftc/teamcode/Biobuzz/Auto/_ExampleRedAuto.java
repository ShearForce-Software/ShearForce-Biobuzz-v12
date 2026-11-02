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
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        CameraSubsystem camera = new CameraSubsystem(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Query camera control bounds and apply settings
        camera.queryCameraControlBoundsAndApply();

        // Target Poses
        Pose startPose = new Pose(60, 135, Math.toRadians(90));
        Pose audienceShootPose = new Pose(49, 130, Math.toRadians(90));
        Pose gardenPose = new Pose(10, 130, Math.toRadians(90));
        Pose backstageShootPose = new Pose(49, 20, Math.toRadians(270));
        Pose flowerPose = new Pose(10, 90, Math.toRadians(180));
        Pose parkPose = new Pose(10, 10, Math.toRadians(90));

        // Set starting pose
        follower.setPose(startPose);

        // Define Path Segments
        Path driveToGarden = line(startPose, gardenPose).linear(startPose, gardenPose);
        Path driveToAudienceShoot1 = line(gardenPose, audienceShootPose).linear(gardenPose, audienceShootPose);
        Path driveToBackstageShoot1 = line(audienceShootPose, backstageShootPose).linear(audienceShootPose, backstageShootPose);
        Path driveToFlower = line(audienceShootPose, flowerPose).linear(audienceShootPose, flowerPose);
        Path driveToAudienceShoot2 = line(flowerPose, audienceShootPose).linear(flowerPose, audienceShootPose);
        Path driveToBackstageShoot2 = line(audienceShootPose, backstageShootPose).linear(audienceShootPose, backstageShootPose);
        Path driveToPark = line(audienceShootPose, parkPose).linear(audienceShootPose, parkPose);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),

                        // 1. Shoot preloaded pollen into audience side red hive on startup
                        shooter.firePositionCommand(),

                        // 2. Drive to the red garden, turning the intake subsystem and transfer subsystem on in parallel
                        parallel(
                                follow(follower, driveToGarden),
                                intake.turnOnCommand(),
                                transfer.rotateForwardCommand()
                        ),

                        // 3. Drive to a shoot position for the audience side red hive, turning the intake subsystem off in parallel
                        parallel(
                                follow(follower, driveToAudienceShoot1),
                                intake.turnOffCommand(),
                                transfer.stopCommand()
                        ),

                        // 4. Use the camera to determine if the red hive is in audience UP position
                        instant(() -> MatchConfig.audienceGoalUp = !Double.isNaN(camera.getCorrectedPedroHeading())),
                        conditional(
                                MatchConfig::isAudienceGoalUp,
                                // IF UP: Shoot at audience hive
                                shooter.firePositionCommand(),
                                // ELSE: Drive to backstage side and shoot
                                sequential(
                                        follow(follower, driveToBackstageShoot1),
                                        shooter.firePositionCommand()
                                )
                        ),

                        // 5. Drive to the audience side red flower, turning on the intake in parallel
                        parallel(
                                follow(follower, driveToFlower),
                                intake.turnOnCommand(),
                                transfer.rotateForwardCommand()
                        ),

                        // 6. Intake more pollen from the flower
                        waitMs(1000),

                        // 7. Drive to the shoot position for the audience side red hive (turning intake/transfer off in parallel)
                        parallel(
                                follow(follower, driveToAudienceShoot2),
                                intake.turnOffCommand(),
                                transfer.stopCommand()
                        ),

                        // 8. Repeat the camera check logic to determine if shoot from audience or backstage side, then shoot
                        instant(() -> MatchConfig.audienceGoalUp = !Double.isNaN(camera.getCorrectedPedroHeading())),
                        conditional(
                                MatchConfig::isAudienceGoalUp,
                                // IF UP: Shoot at audience hive
                                shooter.firePositionCommand(),
                                // ELSE: Drive to backstage side and shoot
                                sequential(
                                        follow(follower, driveToBackstageShoot2),
                                        shooter.firePositionCommand()
                                )
                        ),

                        // 9. Drive to the parking spot and turn off all mechanisms
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

        // Execution loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            Scheduler.execute();
        }

        // Clean up camera on exit
        camera.close();
    }
}
