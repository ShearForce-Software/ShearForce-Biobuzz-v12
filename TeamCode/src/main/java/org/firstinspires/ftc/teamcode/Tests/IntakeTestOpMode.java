package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem.BallColor;

import java.util.Locale;

/**
 * Interactive Test OpMode for the IntakeSubsystem.
 * Displays real-time status for the intake motor, depollinator servo,
 * and Ramp Back / Ramp Front color and distance sensors.
 *
 * Gamepad 1 Controls:
 * - Cross (A) / Right Trigger: Turn on intake motor (spin forward)
 * - Square (X) / Left Trigger: Reverse intake motor (spin backward to eject)
 * - Circle (B): Turn off intake motor
 * - Triangle (Y): Toggle Depollinator Servo between UP and DOWN position
 * - Left Stick Y (Up/Down): Tune Depollinator UP position (+/- 0.05)
 * - Right Stick Y (Up/Down): Tune Depollinator DOWN position (+/- 0.05)
 * - D-Pad Up / Down: Increase / Decrease target intake velocity (+/- 100 ticks/sec)
 */
@TeleOp(name = "Intake Subsystem Test", group = "Testing")
@SuppressWarnings("unused")
public class IntakeTestOpMode extends LinearOpMode {

    // Edge detection for stick-based tuning
    private boolean lastLeftStickUp = false;
    private boolean lastLeftStickDown = false;
    private boolean lastRightStickUp = false;
    private boolean lastRightStickDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Multi-telemetry pushes logs to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        telemetry.addLine("=== Intake Subsystem Test ===");
        telemetry.addLine("Cross / RT: Intake Forward | Square / LT: Reverse | Circle: Off");
        telemetry.addLine("Triangle (Y): Toggle Depollinator Servo UP / DOWN");
        telemetry.addLine("Left/Right Stick Y: Tune Depollinator Servo UP / DOWN Position");
        telemetry.addLine("D-Pad Up/Down: Change Intake Velocity");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Velocity adjustment via D-Pad Up / Down
            if (gamepad1.dpadUpWasPressed()) {
                intakeSubsystem.targetVelocityTicks += 100.0;
            } else if (gamepad1.dpadDownWasPressed()) {
                intakeSubsystem.targetVelocityTicks = Math.max(100.0, intakeSubsystem.targetVelocityTicks - 100.0);
            }

            // Gamepad 1 Motor Controls
            if (gamepad1.cross || gamepad1.right_trigger > 0.1) {
                intakeSubsystem.turnOn();
            } else if (gamepad1.square || gamepad1.left_trigger > 0.1) {
                intakeSubsystem.reverse();
            } else if (gamepad1.circle) {
                intakeSubsystem.turnOff();
            }

            // Depollinator Servo Toggle via Triangle (Y)
            if (gamepad1.triangleWasPressed()) {
                if (intakeSubsystem.isDepollinatorUp()) {
                    intakeSubsystem.depollinatorDown();
                } else {
                    intakeSubsystem.depollinatorUp();
                }
            }

            // Tune Depollinator UP Position using Left Stick Y (0.05 step increments)
            boolean currentLeftStickUp = gamepad1.left_stick_y < -0.1;
            boolean currentLeftStickDown = gamepad1.left_stick_y > 0.1;

            if (currentLeftStickUp && !lastLeftStickUp) {
                IntakeConfig.DEPOLLINATOR_SERVO_UP = Range.clip(IntakeConfig.DEPOLLINATOR_SERVO_UP + 0.05, 0.0, 1.0);
                if (intakeSubsystem.isDepollinatorUp()) intakeSubsystem.depollinatorUp();
            } else if (currentLeftStickDown && !lastLeftStickDown) {
                IntakeConfig.DEPOLLINATOR_SERVO_UP = Range.clip(IntakeConfig.DEPOLLINATOR_SERVO_UP - 0.05, 0.0, 1.0);
                if (intakeSubsystem.isDepollinatorUp()) intakeSubsystem.depollinatorUp();
            }
            lastLeftStickUp = currentLeftStickUp;
            lastLeftStickDown = currentLeftStickDown;

            // Tune Depollinator DOWN Position using Right Stick Y (0.05 step increments)
            boolean currentRightStickUp = gamepad1.right_stick_y < -0.1;
            boolean currentRightStickDown = gamepad1.right_stick_y > 0.1;

            if (currentRightStickUp && !lastRightStickUp) {
                IntakeConfig.DEPOLLINATOR_SERVO_DOWN = Range.clip(IntakeConfig.DEPOLLINATOR_SERVO_DOWN + 0.05, 0.0, 1.0);
                if (intakeSubsystem.isDepollinatorDown()) intakeSubsystem.depollinatorDown();
            } else if (currentRightStickDown && !lastRightStickDown) {
                IntakeConfig.DEPOLLINATOR_SERVO_DOWN = Range.clip(IntakeConfig.DEPOLLINATOR_SERVO_DOWN - 0.05, 0.0, 1.0);
                if (intakeSubsystem.isDepollinatorDown()) intakeSubsystem.depollinatorDown();
            }
            lastRightStickUp = currentRightStickUp;
            lastRightStickDown = currentRightStickDown;

            // Status Telemetry: Motor
            telemetry.addLine("=== Intake Motor ===");
            telemetry.addData("Target Velocity", String.format(Locale.US, "%.0f ticks/s (D-Pad Up/Down)", intakeSubsystem.targetVelocityTicks));
            telemetry.addData("Current Velocity", String.format(Locale.US, "%.1f ticks/s", intakeSubsystem.getVelocity()));

            // Status Telemetry: Depollinator Servo
            telemetry.addLine("=== De-pollinator Servo ===");
            telemetry.addData("Servo State", intakeSubsystem.isDepollinatorUp() ? "UP" : (intakeSubsystem.isDepollinatorDown() ? "DOWN" : "CUSTOM"));
            telemetry.addData("Current Position", String.format(Locale.US, "%.2f", intakeSubsystem.getDepollinatorPosition()));
            telemetry.addData("Configured UP Position", String.format(Locale.US, "%.2f (Left Stick Y)", IntakeConfig.DEPOLLINATOR_SERVO_UP));
            telemetry.addData("Configured DOWN Position", String.format(Locale.US, "%.2f (Right Stick Y)", IntakeConfig.DEPOLLINATOR_SERVO_DOWN));

            // Status Telemetry: Ramp Color & Distance Sensors
            telemetry.addLine("=== Intake Ramp Sensors ===");
            telemetry.addData("Ramp Ball Count", intakeSubsystem.getBallCount() + " / 2");
            telemetry.addData("Ramp Occupied", intakeSubsystem.isRampOccupied() ? "YES" : "NO");

            // Sensor 0: Ramp Back Sensor
            boolean backPresent = intakeSubsystem.isRampBackBallPresent();
            BallColor backColor = intakeSubsystem.detectRampBackColor();
            double backDist = intakeSubsystem.getRampBackDistanceMm();
            NormalizedRGBA backRgba = intakeSubsystem.getRampBackNormalizedColors();
            String backDistStr = (backDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", backDist);
            telemetry.addData("Ramp Back Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    backPresent ? "YES" : "NO ", backColor, backDistStr, backRgba.red, backRgba.green, backRgba.blue));

            // Sensor 1: Ramp Front Sensor
            boolean frontPresent = intakeSubsystem.isRampFrontBallPresent();
            BallColor frontColor = intakeSubsystem.detectRampFrontColor();
            double frontDist = intakeSubsystem.getRampFrontDistanceMm();
            NormalizedRGBA frontRgba = intakeSubsystem.getRampFrontNormalizedColors();
            String frontDistStr = (frontDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", frontDist);
            telemetry.addData("Ramp Front Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    frontPresent ? "YES" : "NO ", frontColor, frontDistStr, frontRgba.red, frontRgba.green, frontRgba.blue));

            telemetry.addLine("=== Controls ===");
            telemetry.addLine("Cross/RT: Turn On | Square/LT: Reverse | Circle: Off");
            telemetry.addLine("Triangle (Y): Toggle Depollinator Servo UP / DOWN");
            telemetry.addLine("Left/Right Stick Y: Tune Depollinator UP / DOWN Position (+/- 0.05)");
            telemetry.addLine("D-Pad Up/Down: Change Intake Velocity");

            telemetry.update();
        }

        intakeSubsystem.turnOff();
    }
}
