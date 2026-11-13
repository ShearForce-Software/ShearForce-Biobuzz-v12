package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
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
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "redTVPlan2", group = "summerPedro")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class RedTVPlan2 extends LinearOpMode {
    //For Sharing-plan-2
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        MatchConfig.allianceColor = MatchConfig.AllianceColor.RED;
        // Instantiate subsystems
        follower = Constants.create(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
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

                        waitMs(8000),
                        shooter.firePositionCommand(),

                        //from start position where you shoot to the flower pose
                        parallel(
                                follow(follower, shootToFlower),
                                intake.turnOnCommand()
                        ),

                        //from the flower turn around and shoot what you intook from the flower
                        follow(follower, flowerToShoot2),
                        waitMs(8000),
                        shooter.firePositionCommand(),










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
