package org.firstinspires.ftc.teamcode.summerPedro.Auto;

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
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.MatchConfig;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "path3pedroYellow", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class path3pedroYellow extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(60, 135, 0);
        Pose collectPollen = new Pose(49, 134, Math.toRadians(0));
        Pose shootPose = new Pose(10, 10, Math.toRadians(90));
        Pose shootPose2 = new Pose(49, 30, Math.toRadians(0));
        Pose drivePose2 = new Pose(26, 30, Math.toRadians(0));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToShoot1 = line(startPose, collectPollen).linear(startPose,collectPollen);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels


                        // Segment 1: Drive to Shoot 1 & Shoot
                        follow(follower, driveToShoot1),
                        waitMs(1000),


                        // Segment 2: Drive to Shoot 2 & Shoot
                       follow(follower, driveToGarden),
                        follow(follower, driveToShoot2),
                       waitMs(1000),
                        follow(follower,retreat),

                        // Segment 3: Drive to opposite side
                        follow(follower, driveToEnd),
                        follow(follower, driveToShoot3),
                        waitMs(1000),
                        follow(follower, retreat2),

                        // Segment 4: Drive to Park
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
