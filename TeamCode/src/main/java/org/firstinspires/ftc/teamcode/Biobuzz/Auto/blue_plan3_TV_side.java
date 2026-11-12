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

@Autonomous(name = "BLUE plan3 TV side", group = "Blue_TV_Side")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class blue_plan3_TV_side extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Record alliance color in MatchConfig
        MatchConfig.allianceColor = MatchConfig.AllianceColor.BLUE;

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(85, 135, 0);
        Pose collectPollen = new Pose(134, 134, Math.toRadians(0));
        Pose shootPose = new Pose(85, 133, Math.toRadians(90));
        Pose collectFlower = new Pose(133, 100, Math.toRadians(90));
        Pose shootPose1 = new Pose(85, 100, Math.toRadians(90));
        Pose driveToPark = new Pose(136, 63, Math.toRadians(180));
        Pose parkPose = new Pose(136, 42, Math.toRadians(0));


        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path collectPollenPath = line(startPose, collectPollen).linear(startPose,collectPollen);
        Path shootPath = line(collectPollen, shootPose).linear(collectPollen,shootPose);
        Path collectFlowerPath = line(shootPose, collectFlower).linear(shootPose,collectFlower);
        Path shootPath1 = line(shootPose1, driveToPark).linear(shootPose1,driveToPark);
        Path park = line(driveToPark, parkPose).linear(driveToPark,parkPose);


        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),
                        // Spin up the flywheels


                        // Segment 1: shoot and collect from garden
                        waitMs(1000),
                        follow(follower, collectPollenPath),
                        waitMs(4000),

                        // Segment 2: Drive to Shoot & collect again
                       follow(follower, shootPath),
                        waitMs(1000),
                        follow(follower, collectFlowerPath),
                        waitMs(5300),

                        // Segment 3: shoot & Park
                        follow(follower, shootPath1),
                        follow(follower, park),


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
