package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterConfig;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterSubsystem;

import java.util.Locale;

/*
OpMode for live PIDF tuning and graph plotting of the dual flywheel shooter system via FTC Dashboard.

Target Hardware:
- Motors: goBilda 5203 Series 1:1 Yellow Jacket (6000 RPM max, 28 CPR)
- Initial Target: 1750 RPM = 816.7 ticks/sec

FTC Dashboard Instructions:
1. Connect laptop to Robot Wi-Fi.
2. Open Browser: http://192.168.43.1:8080/dash
3. Open Graph Panel and enable:
   - Target Velocity
   - Left Velocity
   - Right Velocity
   - Average Velocity
4. Edit PIDF values in 'ShooterConfig' (press Enter or click Save Config to apply):
   a. F = 11.7 (Feedforward initialized for 1:1 motor)
   b. Increase P until speed reaches target quickly without excessive overshoot.
   c. Increase D to dampen overshoot / oscillations.
   d. Add small I if steady-state error remains under load.
5. Driver Controls (Gamepad 1):
   - Cross / Right Trigger: Actuate pusher servo to feed ball and observe speed recovery.
   - Circle: Toggle flywheels ON / OFF.
   - D-Pad Up: Increment target velocity (+50 ticks/sec).
   - D-Pad Down: Decrement target velocity (-50 ticks/sec).
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    private static final double VELOCITY_STEP_TICKS = 50.0; // ~107 RPM step change

    private boolean lastCircleState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;

    private double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wrap Driver Station telemetry and FTC Dashboard telemetry together
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("=== Shooter PIDF Tuner ===");
        telemetry.addLine("Open http://192.168.43.1:8080/dash");
        telemetry.addLine("Edit values in category: ShooterConfig (press Enter/Save to send)");
        telemetry.addLine("Press Play to start");
        telemetry.addLine("--------------------------------------");
        telemetry.addLine("Right Trigger: Feed ball");
        telemetry.addLine("Circle: Toggle flywheels on/off");
        telemetry.addLine("Left/Right Bumpers: Decrease/Increase target velocity by increments");
        telemetry.addLine("Triangle Button: Toggle Slow/Fast target velocity");
        telemetry.addLine("Square Button: Change PIDF step size");
        telemetry.addLine("D-Pad Up/Down: Increase/Decrease PIDF P value");
        telemetry.addLine("D-Pad Left/Right: Increase/Decrease PIDF F value");
        telemetry.update();

        waitForStart();

        // Force the dashboard to push its current web-client values into your code's memory registers
        FtcDashboard.getInstance().updateConfig();

        // Start flywheels initially
        shooter.startFlywheels();

        while (opModeIsActive()) {
            // Re-apply PIDF gains and target velocity if modified live on FTC Dashboard
            shooter.update();

            // Feed ball control: Cross button or Right Trigger
            if (gamepad1.right_trigger > 0.1) {
                shooter.firePosition();
            } else {
                shooter.restPosition();
            }

            // Circle button toggles flywheels on/off
            if (gamepad1.circleWasPressed()) {
                if (shooter.isRunning()) {
                    shooter.stopFlywheels();
                } else {
                    shooter.startFlywheels();
                }
            }

            // Manually adjust the Feed Forward value
            if (gamepad1.dpadLeftWasPressed()) {
                ShooterConfig.F += stepSizes[stepIndex];
            }
            else if (gamepad1.dpadRightWasPressed()) {
                ShooterConfig.F -= stepSizes[stepIndex];
            }

            // Manually adjust the Proportional Gain value
            if (gamepad1.dpadUpWasPressed()) {
                ShooterConfig.P += stepSizes[stepIndex];
            }
            else if (gamepad1.dpadDownWasPressed()) {
                ShooterConfig.P -= stepSizes[stepIndex];

            }

            // Use the square button to change the step size of the PIDF adjustment
            if (gamepad1.squareWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            // use the triangle button to toggle the target velocity between slow and fast
            if (gamepad1.triangleWasPressed()) {
               if (shooter.getTargetVelocity() >= (ShooterConfig.DEFAULT_HIGH_VELOCITY - ShooterConfig.VELOCITY_TOLERANCE)) {
                   shooter.setVelocity(ShooterConfig.DEFAULT_LOW_VELOCITY);
               } else {
                   shooter.setVelocity(ShooterConfig.DEFAULT_HIGH_VELOCITY);
               }
            }

            // Left and Right Bumpers adjusts target velocity in increments
            if (gamepad1.rightBumperWasPressed()) {
                double newVel = shooter.getTargetVelocity() + VELOCITY_STEP_TICKS;
                shooter.setVelocity(newVel);
            }
            else if (gamepad1.leftBumperWasPressed()) {
                double newVel = Math.max(0.0, shooter.getTargetVelocity() - VELOCITY_STEP_TICKS);
                shooter.setVelocity(newVel);
            }

            // Format "At Speed" message with last time to speed / recovery duration
            StringBuilder atSpeedMsg = new StringBuilder(shooter.isAtSpeed() ? "TRUE" : "FALSE");
            if (shooter.getLastTimeToSpeed() >= 0) {
                atSpeedMsg.append(String.format(Locale.US, " (time to speed: %.3f s )",
                        shooter.getLastTimeToSpeed()));
            }

            // Format "Flywheels Running" message with last 0-to-speed startup duration
            StringBuilder flywheelsMsg = new StringBuilder(shooter.isRunning() ? "RUNNING" : "STOPPED");
            if (shooter.getLastStartupTime() >= 0) {
                flywheelsMsg.append(String.format(Locale.US, " (startup time: %.3f s )",
                        shooter.getLastStartupTime()));
            }

            // Calculate RPM from encoder ticks/sec: RPM = (ticksPerSec * 60 seconds/min) / ticksPerRev
            double targetRpm = (shooter.getTargetVelocity() * 60.0) / ShooterConfig.MOTOR_ENCODER_TICKS_PER_REVOLUTION;
            double averageRpm = (shooter.getAverageVelocity() * 60.0) / ShooterConfig.MOTOR_ENCODER_TICKS_PER_REVOLUTION;

            // Calculate the current velocity error
            double currentVelocityError = shooter.getTargetVelocity() - shooter.getAverageVelocity();
            double currentRPMError = targetRpm - averageRpm;

            // Broadcast telemetry to Driver Station and Dashboard Graph
            telemetry.addData("Target Velocity", "%.1f (Bumpers or Triangle)", shooter.getTargetVelocity());
            telemetry.addData("Average Velocity", "%.1f", shooter.getAverageVelocity());
            telemetry.addData("Target / Average RPM", "Target: %.1f | Avg: %.1f", targetRpm, averageRpm);
            telemetry.addData("Left / Right Velocity", "L: %.1f | R: %.1f", shooter.getLeftVelocity(), shooter.getRightVelocity());
            telemetry.addData("Velocity Error", "%.2f ticks/sec (%.2f RPM)", currentVelocityError, currentRPMError);
            telemetry.addData("At Speed", atSpeedMsg.toString());
            telemetry.addData("Flywheels Running", flywheelsMsg.toString());
            telemetry.addData("PIDF ", "P: %.4f (DPAD UP/DOWN)", ShooterConfig.P);
            telemetry.addData("PIDF ", "F: %.2f (DPAD LEFT/RIGHT)", ShooterConfig.F);
            telemetry.addData("Step Size", "%.4f (SQUARE)", stepSizes[stepIndex]);

            telemetry.addLine("--------------------------------------");
            telemetry.addLine("=== Shooter PIDF Tuner CONTROLS ===");
            telemetry.addLine("Open http://192.168.43.1:8080/dash");
            telemetry.addLine("Edit values in category: ShooterConfig (press Enter/Save to send)");
            telemetry.addLine("Right Trigger: Feed ball");
            telemetry.addLine("Circle: Toggle flywheels on/off");
            telemetry.addLine("Left/Right Bumpers: Decrease/Increase target velocity by increments");
            telemetry.addLine("Triangle Button: Toggle Slow/Fast target velocity");
            telemetry.addLine("Square Button: Change PIDF step size");
            telemetry.addLine("D-Pad Up/Down: Increase/Decrease PIDF P value");
            telemetry.addLine("D-Pad Left/Right: Increase/Decrease PIDF F value");

            telemetry.update();
        }

        shooter.stopFlywheels();
    }
}
