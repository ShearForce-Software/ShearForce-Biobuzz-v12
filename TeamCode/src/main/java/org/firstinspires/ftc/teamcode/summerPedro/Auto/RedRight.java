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

@Autonomous(name = "redRight", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class RedRight extends LinearOpMode {
    //For Sharing-plan-2
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(60, 9, Math.toRadians(90));
        Pose gardenControlPoint = new Pose(41, 19, Math.toRadians(180));
        Pose garden = new Pose(22, 9, Math.toRadians(180));
        Pose collectGarden = new Pose(9, 9, Math.toRadians(180));
        Pose gardenToShootControl = new Pose(33, 18, Math.toRadians(90));
        Pose shootPose = new Pose(60, 9, Math.toRadians(90));
        Pose flower = new Pose(10, 48, Math.toRadians(180));


        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path driveToGarden = curve(startPose, gardenControlPoint, garden).linear(startPose, garden);
        Path collectFromGarden = line(garden, collectGarden).linear(garden, collectGarden);
        Path leaveGarden = line(collectGarden, garden).linear(collectGarden, garden);
        Path driveToShoot = curve(garden, gardenToShootControl, shootPose).linear(collectGarden, shootPose);
        Path driveToFlower = line(shootPose, flower).linear(shootPose, flower);
        Path flowerToShoot = line(flower, shootPose).linear(flower, shootPose);


        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),

                        // Segment 1: Shoot preload and drive to garden and collect pollen
                        waitMs(1500),
                        follow(follower, driveToGarden),
                        follow(follower, collectFromGarden),
                        waitMs(500),


                        // Segment 2: Drive out of garden and back to shoot pose
                        follow(follower, leaveGarden),
                        follow(follower, driveToShoot),
                        waitMs(1000),


                        // Segment 3: drive to the flower and collect pollen then drive back to the shoot pose
                        follow(follower, driveToFlower),
                        waitMs(1000),
                        follow(follower, flowerToShoot),


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
