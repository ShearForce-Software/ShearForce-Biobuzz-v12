package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

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

import org.firstinspires.ftc.teamcode.Biobuzz.pedro.Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "redAudiencePlan2", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class RedTVPlan2 extends LinearOpMode {
    //For Sharing-plan-2
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(63, 135, Math.toRadians(-90));
        Pose controlForFlower = new Pose(55.5, 100, Math.toRadians(90));
        Pose flower = new Pose(48,135, Math.toRadians(90));
        Pose shoot2 = new Pose(48, 120, Math.toRadians(-70));
        Pose hive1 = new Pose(55, 95, Math.toRadians(-90));
        Pose hive2 = new Pose (59, 61.5, Math.toRadians(-90));
        Pose hive3 = new Pose (59, 103, Math.toRadians(-90));
        Pose loadingZone = new Pose(12, 107, Math.toRadians(-30));


        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path shootToFlower = curve(startPose, controlForFlower, flower).linear(startPose, flower);
        Path flowerToShoot2 = line(flower, shoot2).linear(flower, shoot2);
        Path shoot2Tohive1 = line(shoot2, hive1).linear(shoot2, hive1);
        Path hive1ToHive2 = line(hive1, hive2).linear(hive1, hive2);
        Path hive2TOhive3 = line(hive2, hive3).linear(hive2, hive3);
        Path hive3ToLoadingZone = line(hive3, loadingZone).linear(hive3, loadingZone);


        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),

                        // Segment 1: Shoot preload and drive to garden and collect pollen



                        // Segment 2: Drive out of garden and back to shoot pose



                        // Segment 3: drive to the flower and collect pollen then drive back to the shoot pose



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
