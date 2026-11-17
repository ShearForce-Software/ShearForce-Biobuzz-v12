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
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

// Pedro 3 + Ivy Core Framework Imports
// Static Ivy Command Structure Imports

@Autonomous(name = "redTVSimplePark", group = "summerPedro", preselectTeleOp = "BIOBUZZ TeleOp")
@SuppressWarnings("unused") // Eliminates the "class never used" warning
public class RedTVSimplePark extends LinearOpMode {
    //For Sharing-plan-2
    private Follower follower;
    @Override
    public void runOpMode() {
        Scheduler.reset();

        MatchConfig.allianceColor = MatchConfig.AllianceColor.RED;
        // Instantiate subsystems
        follower = Constants.create(hardwareMap);

        ElapsedTime matchTimer = new ElapsedTime();

        // Target Poses
        Pose startPose = new Pose(15, 135, Math.toRadians(-90));
        Pose parkPose = new Pose(12, 96, Math.toRadians(-90));

        // Set starting pose
        follower.setPose(startPose);

        // Pedro 3 Path API: Paths replace PathChain completely
        Path startToPark = line(startPose, parkPose).linear(startPose, parkPose);



        // Build the Ivy Autonomous Command Sequence
        schedule(
                sequential(
                        instant(matchTimer::reset),

                        follow(follower, startToPark),


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
