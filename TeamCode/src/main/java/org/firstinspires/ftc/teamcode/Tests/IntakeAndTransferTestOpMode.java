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
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

import java.util.Locale;

/**
 * Interactive Combined Test and Tuning OpMode for the IntakeSubsystem and TransferSubsystem.
 *
 * Gamepad 1 Controls (INTAKE SUBSYSTEM):
 * - Cross (A) / Right Trigger: Turn on intake motor (spin forward)
 * - Square (X) / Left Trigger: Reverse intake motor (spin backward to eject)
 * - Circle (B): Turn off intake motor
 * - Triangle (Y): Toggle Depollinator Servo between UP and DOWN position
 * - Left Stick Y (Up/Down): Tune Depollinator UP position (+/- 0.05)
 * - Right Stick Y (Up/Down): Tune Depollinator DOWN position (+/- 0.05)
 * - D-Pad Up / Down: Increase / Decrease target intake velocity (+/- 100 ticks/sec)
 *
 * Gamepad 2 Controls (TRANSFER SUBSYSTEM):
 * - Cross (A) / Right Trigger: Rotate transfer forward continuously
 * - Square (X) / Left Trigger: Rotate transfer in reverse continuously
 * - Circle (B): Stop transfer rotation
 * - Right Bumper: Advance 1/4 rotation (1 slot) forward
 * - Left Bumper: Advance 1/4 rotation (1 slot) in reverse
 * - Triangle (Y): Reset REV Through Bore Encoder position
 * - D-Pad Up / Down: Increase / Decrease custom transfer speed (+/- 0.1)
 * - D-Pad Left / Right: Decrease / Increase TransferConfig.TICKS_PER_REV (+/- 100)
 */
@TeleOp(name = "Intake & Transfer Combined Test", group = "Testing")
@SuppressWarnings("unused")
public class IntakeAndTransferTestOpMode extends LinearOpMode {
    private double customTransferSpeed = Math.abs(TransferConfig.FORWARD_POWER);

    // Edge detection for Gamepad 1 stick-based tuning
    private boolean lastLeftStickUp = false;
    private boolean lastLeftStickDown = false;
    private boolean lastRightStickUp = false;
    private boolean lastRightStickDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Multi-telemetry pushes logs to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transferSubsystem = new TransferSubsystem(hardwareMap);

        telemetry.addLine("=== Intake & Transfer Combined Test ===");
        telemetry.addLine("G1 (Intake): Cross/RT = On | Square/LT = Reverse | Circle = Off");
        telemetry.addLine("G1 (Depollinator): Triangle = Toggle UP/DOWN | Left/Right Stick Y = Tune");
        telemetry.addLine("G2 (Transfer): Cross/RT = Fwd | Square/LT = Rev | Bumpers = 1-Slot");
        telemetry.addLine("G2 (Transfer Tuning): D-Pad Up/Down = Speed | D-Pad Left/Right = Ticks/Rev");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================================================================
            // GAMEPAD 1 CONTROLS: INTAKE SUBSYSTEM
            // =========================================================================

            // Intake Velocity adjustment via Gamepad 1 D-Pad Up / Down
            if (gamepad1.dpadUpWasPressed()) {
                intakeSubsystem.targetVelocityTicks += 100.0;
            } else if (gamepad1.dpadDownWasPressed()) {
                intakeSubsystem.targetVelocityTicks = Math.max(100.0, intakeSubsystem.targetVelocityTicks - 100.0);
            }

            // Gamepad 1 Intake Motor Controls
            if (gamepad1.cross || gamepad1.right_trigger > 0.1) {
                intakeSubsystem.turnOn();
            } else if (gamepad1.square || gamepad1.left_trigger > 0.1) {
                intakeSubsystem.reverse();
            } else if (gamepad1.circle) {
                intakeSubsystem.turnOff();
            }

            // Depollinator Servo Toggle via Gamepad 1 Triangle (Y)
            if (gamepad1.triangleWasPressed()) {
                if (intakeSubsystem.isDepollinatorUp()) {
                    intakeSubsystem.depollinatorDown();
                } else {
                    intakeSubsystem.depollinatorUp();
                }
            }

            // Tune Depollinator UP Position using Gamepad 1 Left Stick Y (0.05 step increments)
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

            // Tune Depollinator DOWN Position using Gamepad 1 Right Stick Y (0.05 step increments)
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

            // =========================================================================
            // GAMEPAD 2 CONTROLS: TRANSFER SUBSYSTEM
            // =========================================================================

            // Speed adjustment via Gamepad 2 D-Pad Up / Down
            if (gamepad2.dpadUpWasPressed()) {
                customTransferSpeed = Math.min(1.0, customTransferSpeed + 0.1);
            } else if (gamepad2.dpadDownWasPressed()) {
                customTransferSpeed = Math.max(0.1, customTransferSpeed - 0.1);
            }

            // TICKS_PER_REV adjustment via Gamepad 2 D-Pad Left / Right
            if (gamepad2.dpadLeftWasPressed()) {
                TransferConfig.TICKS_PER_REV = Math.max(100.0, TransferConfig.TICKS_PER_REV - 100.0);
            } else if (gamepad2.dpadRightWasPressed()) {
                TransferConfig.TICKS_PER_REV += 100.0;
            }

            // Servo Rotation Controls
            if (gamepad2.cross || gamepad2.right_trigger > 0.1) {
                transferSubsystem.setPower(-customTransferSpeed);
            } else if (gamepad2.square || gamepad2.left_trigger > 0.1) {
                transferSubsystem.setPower(customTransferSpeed);
            } else if (gamepad2.circle) {
                transferSubsystem.stop();
            }

            // Advance 1/4 full rotation (1 slot) via Gamepad 2 Right / Left Bumpers
            if (gamepad2.rightBumperWasPressed()) {
                transferSubsystem.Shoot1Ball();
            } else if (gamepad2.leftBumperWasPressed()) {
                transferSubsystem.Load1Ball();
            }

            // Reset Through Bore Encoder Position via Gamepad 2 Triangle
            if (gamepad2.triangleWasPressed()) {
                transferSubsystem.resetEncoder();
            }

            // Update active transfer subsystem state transitions (e.g. 1-slot advancement)
            transferSubsystem.update();

            // =========================================================================
            // TELEMETRY OUTPUTS: INTAKE SUBSYSTEM
            // =========================================================================

            telemetry.addLine("========= INTAKE SUBSYSTEM (G1) =========");
            telemetry.addData("Intake Target Velocity", String.format(Locale.US, "%.0f ticks/s (G1 D-Pad Up/Down)", intakeSubsystem.targetVelocityTicks));
            telemetry.addData("Intake Current Velocity", String.format(Locale.US, "%.1f ticks/s", intakeSubsystem.getVelocity()));

            telemetry.addData("Depollinator State", intakeSubsystem.isDepollinatorUp() ? "UP" : (intakeSubsystem.isDepollinatorDown() ? "DOWN" : "CUSTOM"));
            telemetry.addData("Depollinator Position", String.format(Locale.US, "%.2f (Config UP: %.2f | DOWN: %.2f)",
                    intakeSubsystem.getDepollinatorPosition(), IntakeConfig.DEPOLLINATOR_SERVO_UP, IntakeConfig.DEPOLLINATOR_SERVO_DOWN));

            telemetry.addData("Ramp Ball Count", intakeSubsystem.getBallCount() + " / 2");

            boolean backPresent = intakeSubsystem.isRampBackBallPresent();
            BallColor backColor = intakeSubsystem.detectRampBackColor();
            double backDist = intakeSubsystem.getRampBackDistanceMm();
            NormalizedRGBA backRgba = intakeSubsystem.getRampBackNormalizedColors();
            String backDistStr = (backDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", backDist);
            telemetry.addData("  └ Ramp Back Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    backPresent ? "YES" : "NO ", backColor, backDistStr, backRgba.red, backRgba.green, backRgba.blue));

            boolean frontPresent = intakeSubsystem.isRampFrontBallPresent();
            BallColor frontColor = intakeSubsystem.detectRampFrontColor();
            double frontDist = intakeSubsystem.getRampFrontDistanceMm();
            NormalizedRGBA frontRgba = intakeSubsystem.getRampFrontNormalizedColors();
            String frontDistStr = (frontDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", frontDist);
            telemetry.addData("  └ Ramp Front Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    frontPresent ? "YES" : "NO ", frontColor, frontDistStr, frontRgba.red, frontRgba.green, frontRgba.blue));

            // =========================================================================
            // TELEMETRY OUTPUTS: TRANSFER SUBSYSTEM
            // =========================================================================

            telemetry.addLine("========= TRANSFER SUBSYSTEM (G2) =========");
            telemetry.addData("Transfer Servo Power", String.format(Locale.US, "%.2f (Speed Setting: %.2f)", transferSubsystem.getPower(), customTransferSpeed));
            telemetry.addData("Encoder Position", String.format(Locale.US, "%.0f ticks (%.3f rev | %.1f deg) [Ticks/Rev: %.0f]",
                    transferSubsystem.getRawTicks(), transferSubsystem.getRotations(), transferSubsystem.getDegrees(), TransferConfig.TICKS_PER_REV));
            telemetry.addData("Encoder Velocity", String.format(Locale.US, "%.1f ticks/s (%.1f RPM)",
                    transferSubsystem.getVelocityTicksPerSec(), transferSubsystem.getVelocityRPM()));

            telemetry.addData("Transfer Ball Count", transferSubsystem.getBallCount() + " / 4 (" + (transferSubsystem.isFull() ? "FULL" : (transferSubsystem.isEmpty() ? "EMPTY" : "PARTIAL")) + ")");

            // Slot 0: Top Slot
            boolean topPresent = transferSubsystem.isSlotOccupied(TransferSubsystem.SENSOR_TOP);
            TransferSubsystem.BallColor topColor = transferSubsystem.detectSlotColor(TransferSubsystem.SENSOR_TOP);
            double topDist = transferSubsystem.getDistanceMm(TransferSubsystem.SENSOR_TOP);
            NormalizedRGBA topRgba = transferSubsystem.getNormalizedColors(TransferSubsystem.SENSOR_TOP);
            String topDistStr = (topDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", topDist);
            telemetry.addData("  └ Top Slot", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    topPresent ? "YES" : "NO ", topColor, topDistStr, topRgba.red, topRgba.green, topRgba.blue));

            // Slot 1: Left Slot
            boolean leftPresent = transferSubsystem.isSlotOccupied(TransferSubsystem.SENSOR_LEFT);
            TransferSubsystem.BallColor leftColor = transferSubsystem.detectSlotColor(TransferSubsystem.SENSOR_LEFT);
            double leftDist = transferSubsystem.getDistanceMm(TransferSubsystem.SENSOR_LEFT);
            NormalizedRGBA leftRgba = transferSubsystem.getNormalizedColors(TransferSubsystem.SENSOR_LEFT);
            String leftDistStr = (leftDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", leftDist);
            telemetry.addData("  └ Left Slot", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    leftPresent ? "YES" : "NO ", leftColor, leftDistStr, leftRgba.red, leftRgba.green, leftRgba.blue));

            // Slot 2: Right Slot
            boolean rightPresent = transferSubsystem.isSlotOccupied(TransferSubsystem.SENSOR_RIGHT);
            TransferSubsystem.BallColor rightColor = transferSubsystem.detectSlotColor(TransferSubsystem.SENSOR_RIGHT);
            double rightDist = transferSubsystem.getDistanceMm(TransferSubsystem.SENSOR_RIGHT);
            NormalizedRGBA rightRgba = transferSubsystem.getNormalizedColors(TransferSubsystem.SENSOR_RIGHT);
            String rightDistStr = (rightDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rightDist);
            telemetry.addData("  └ Right Slot", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    rightPresent ? "YES" : "NO ", rightColor, rightDistStr, rightRgba.red, rightRgba.green, rightRgba.blue));

            // Slot 3: Bottom Slot
            boolean bottomPresent = transferSubsystem.isSlotOccupied(TransferSubsystem.Slot.BOTTOM);
            TransferSubsystem.BallColor bottomColor = transferSubsystem.detectSlotColor(TransferSubsystem.Slot.BOTTOM);
            telemetry.addData("  └ Bottom Slot", String.format(Locale.US, "Present: %s | Color: %s", bottomPresent ? "YES" : "NO ", bottomColor));

            boolean bbPresent = transferSubsystem.isSensorDetectingBall(TransferSubsystem.SENSOR_BOTTOM_BACK);
            TransferSubsystem.BallColor bbColor = transferSubsystem.detectSensorColor(TransferSubsystem.SENSOR_BOTTOM_BACK);
            double bbDist = transferSubsystem.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_BACK);
            NormalizedRGBA bbRgba = transferSubsystem.getNormalizedColors(TransferSubsystem.SENSOR_BOTTOM_BACK);
            String bbDistStr = (bbDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", bbDist);
            telemetry.addData("      └ Bottom Back Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    bbPresent ? "YES" : "NO ", bbColor, bbDistStr, bbRgba.red, bbRgba.green, bbRgba.blue));

            boolean bfPresent = transferSubsystem.isSensorDetectingBall(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            TransferSubsystem.BallColor bfColor = transferSubsystem.detectSensorColor(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            double bfDist = transferSubsystem.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            NormalizedRGBA bfRgba = transferSubsystem.getNormalizedColors(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            String bfDistStr = (bfDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", bfDist);
            telemetry.addData("      └ Bottom Front Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    bfPresent ? "YES" : "NO ", bfColor, bfDistStr, bfRgba.red, bfRgba.green, bfRgba.blue));

            telemetry.addLine("=========================================");
            telemetry.addLine("G1 Controls: Cross/RT = Intake | Square/LT = Reverse | Triangle = Depollinator | D-Pad Up/Down = Vel");
            telemetry.addLine("G2 Controls: Cross/RT = Fwd | Square/LT = Rev | Bumpers = 1-Slot | D-Pad Up/Down = Speed | D-Pad L/R = Ticks");

            telemetry.update();
        }

        intakeSubsystem.turnOff();
        transferSubsystem.stop();
    }
}
