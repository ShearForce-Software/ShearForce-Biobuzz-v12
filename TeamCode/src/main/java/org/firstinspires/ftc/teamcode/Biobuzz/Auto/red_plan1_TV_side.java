package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

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

import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "RED plan1 TV Side", group = "RedTVSide")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class red_plan1_TV_side extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Record alliance color in MatchConfig
        MatchConfig.allianceColor = MatchConfig.AllianceColor.RED;

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(58, 124, 0);
        Pose flower = new Pose(49, 131, Math.toRadians(0));
        Pose shootPose = new Pose(58, 124, Math.toRadians(0));
        Pose park = new Pose(15, 115, Math.toRadians(90));
        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path goToFlower = line(startPose, flower).linear(startPose, flower);
        Path backToShootPose = line(flower, shootPose).linear(flower, shootPose);
        Path driveToPark = line(flower, park).linear(flower, park);
        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // shoot 1
                        waitMs(2000),

                        // go to flower
                        follow(follower, goToFlower),

                        // collect from flower
                        waitMs(2000),

                        // back to shooting pose
                        follow(follower, backToShootPose),

                        // shoot 2
                        waitMs(2000),

                        // park
                        follow(follower, driveToPark),

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
