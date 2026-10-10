package org.firstinspires.ftc.teamcode.summerPedro.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import org.firstinspires.ftc.teamcode.summerPedro.pedro.Constants;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.MatchConfig;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterSubsystem;

@TeleOp(name = "summerPedro TeleOp", group = "summerPedro")
public class PedroExampleTeleOp extends OpMode {
    private Follower follower;
    private Command holdCommand = null; // Store the command reference to allow cancelling
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    private boolean isHolding = false;
    private boolean lastCrossState = false; // Tracks button state for toggle

    @Override
    public void init() {
        // Initialize Ivy Scheduler and Pedro 3 Follower
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Restore exact ending coordinate Pose from Autonomous
        follower.setPose(MatchConfig.currentPose);
    }

    @Override
    public void loop() {
        // Read joystick inputs (invert y-axis for FTC standard forward = negative)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Break out of position hold if significant driver input is detected
        boolean driverInputDetected = Math.hypot(forward, strafe) > 0.05 || Math.abs(turn) > 0.05;
        if (driverInputDetected && isHolding) {
            isHolding = false;
            if (holdCommand != null) {
                holdCommand.cancel(); // Correct Ivy API: cancel individual command object
                holdCommand = null;
            }
        }

        // Toggle hold mode using the 'A' / Cross button
        boolean currentCrossState = gamepad1.a;
        if (currentCrossState && !lastCrossState) {
            isHolding = !isHolding;
            if (isHolding) {
                // Generate and store the hold command instance, then schedule it
                holdCommand = hold(follower);
                schedule(holdCommand);
            } else {
                if (holdCommand != null) {
                    holdCommand.cancel(); // Correct Ivy API: cancel individual command object
                    holdCommand = null;
                }
            }
        }
        lastCrossState = currentCrossState;

        // Handle driving modes
        if (isHolding) {
            // Run the Ivy scheduler pipeline to execute the hold loop
            Scheduler.execute();
        } else {
            // Field-centric vector calculation
            double robotHeading = follower.pose().heading();

            // Perform standard field-centric vector rotation matrix
            final double cos = Math.cos(-robotHeading);
            final double sin = Math.sin(-robotHeading);
            double fieldCentricForward = forward * cos - strafe * sin;
            double fieldCentricStrafe = forward * sin + strafe * cos;

            // Feeding values directly into manual() sets the follower state to MANUAL
            follower.manual(fieldCentricForward, fieldCentricStrafe, turn);
        }

        // Operator Mechanism Controls
        // Intake: Right Trigger to Intake, Left Trigger to Reverse
        if (gamepad2.right_trigger > 0.1) {
            intake.turnOn();
        } else if (gamepad2.left_trigger > 0.1) {
            intake.reverse();
        } else {
            intake.turnOff();
        }

        // Flywheel Spool: Y toggles on/off or spin-up
        if (gamepad2.y) {
            shooter.startFlywheels();
        } else if (gamepad2.b) {
            shooter.stopFlywheels();
        }

        // Fire Indexer: A button fires indexer once flywheels are at target velocity
        if (gamepad2.a && shooter.isAtSpeed()) {
            shooter.firePosition();
        } else {
            shooter.restPosition();
        }

        // Essential: Keep Pedro's odometry tracking and internal path loops refreshing
        follower.update();
        // Update the shooter PIDF (from Dashboard) if needed
        shooter.update();

        // Diagnostics
        telemetry.addData("Currently Holding Point", isHolding);
        telemetry.addData("Shooter Velocity", "%.1f / %.1f", shooter.getAverageVelocity(), shooter.getTargetVelocity());
        telemetry.addData("Shooter Ready", shooter.isAtSpeed());
        telemetry.addData("Pose", follower.pose().toString());
        telemetry.update();
    }
}
