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
        Pose collectPollen = new Pose(49, 134, Math.toRadians(90));
        Pose shootPose = new Pose(60, 134, Math.toRadians(90));
        Pose oppositeShoot = new Pose(60, 12, Math.toRadians(180));
        Pose drivePose = new Pose(60, 120, Math.toRadians(0));
        Pose drivePose1 = new Pose(10, 120, Math.toRadians(0));
        Pose parkPose = new Pose(10, 107, Math.toRadians(0));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToShoot1 = line(startPose, collectPollen).linear(startPose,collectPollen);
        Path collectPollenDrive = line(collectPollen, shootPose).linear(collectPollen,shootPose);
        Path oppositeSide = line(shootPose, oppositeShoot).linear(shootPose,oppositeShoot);
        Path retreat = line(oppositeShoot, drivePose).linear(oppositeShoot,drivePose);
        Path readyToPark = line(drivePose, drivePose1).linear(drivePose,drivePose1);
        Path Park = line(drivePose1, parkPose).linear(drivePose1,parkPose);

        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels


                        // Segment 1: Drive to Shoot 1 & Collect
                        waitMs(1000),
                        follow(follower, collectPollenDrive),
                        waitMs(4000),

                        // Segment 2: Drive to Shoot & Return
                       follow(follower, oppositeSide),
                        waitMs(1000),
                        follow(follower, retreat),

                        // Segment 3: Park
                        follow(follower, readyToPark),
                        follow(follower, Park),


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
