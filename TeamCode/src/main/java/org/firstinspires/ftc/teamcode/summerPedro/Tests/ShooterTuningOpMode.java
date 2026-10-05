package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterSubsystem;

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
4. Edit PIDF values in 'summerPedro -> ShooterConfig':
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

    @Override
    public void runOpMode() throws InterruptedException {
        // Wrap Driver Station telemetry and FTC Dashboard telemetry together
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("=== Shooter PIDF Tuner ===");
        telemetry.addLine("Open http://192.168.43.1:8080/dash");
        telemetry.addLine("Edit values in summerPedro -> ShooterConfig");
        telemetry.addLine("Cross / Right Trigger: Feed ball");
        telemetry.addLine("Circle: Toggle flywheels on/off");
        telemetry.addLine("D-Pad Up/Down: Increase/Decrease target velocity");
        telemetry.update();

        waitForStart();

        // Start flywheels initially
        shooter.startFlywheels();

        while (opModeIsActive()) {
            // Re-apply PIDF gains and target velocity if modified live on FTC Dashboard
            shooter.update();

            // Feed ball control: Cross button or Right Trigger
            if (gamepad1.cross || gamepad1.right_trigger > 0.1) {
                shooter.firePosition();
            } else {
                shooter.restPosition();
            }

            // Circle button toggles flywheels on/off
            boolean currentCircleState = gamepad1.circle;
            if (currentCircleState && !lastCircleState) {
                if (shooter.isRunning()) {
                    shooter.stopFlywheels();
                } else {
                    shooter.startFlywheels();
                }
            }
            lastCircleState = currentCircleState;

            // D-Pad Up / Down adjusts target velocity in increments
            boolean currentDpadUpState = gamepad1.dpad_up;
            if (currentDpadUpState && !lastDpadUpState) {
                double newVel = shooter.getTargetVelocity() + VELOCITY_STEP_TICKS;
                shooter.setVelocity(newVel);
            }
            lastDpadUpState = currentDpadUpState;

            boolean currentDpadDownState = gamepad1.dpad_down;
            if (currentDpadDownState && !lastDpadDownState) {
                double newVel = Math.max(0.0, shooter.getTargetVelocity() - VELOCITY_STEP_TICKS);
                shooter.setVelocity(newVel);
            }
            lastDpadDownState = currentDpadDownState;

            // Broadcast telemetry to Driver Station and Dashboard Graph
            telemetry.addData("Target Velocity", shooter.getTargetVelocity());
            telemetry.addData("Left Velocity", shooter.getLeftVelocity());
            telemetry.addData("Right Velocity", shooter.getRightVelocity());
            telemetry.addData("Average Velocity", shooter.getAverageVelocity());
            telemetry.addData("At Speed", shooter.isAtSpeed());
            telemetry.addData("Flywheels Running", shooter.isRunning());

            telemetry.update();
        }

        shooter.stopFlywheels();
    }
}
