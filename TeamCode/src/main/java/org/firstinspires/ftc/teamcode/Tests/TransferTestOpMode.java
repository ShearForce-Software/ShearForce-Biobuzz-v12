package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem.BallColor;

import java.util.Locale;

/**
 * Interactive Test and Tuning OpMode for the TransferSubsystem.
 * Displays real-time slot occupancy and sensor data for Top, Left, Right,
 * and Bottom (Bottom Back & Bottom Front) slots.
 *
 * Gamepad 1 Controls:
 * - Cross (A) / Right Trigger: Rotate transfer forward continuously
 * - Square (X) / Left Trigger: Rotate transfer in reverse continuously
 * - Circle (B): Stop transfer rotation
 * - Right Bumper: Advance 1/4 rotation (1 slot) forward
 * - Left Bumper: Advance 1/4 rotation (1 slot) in reverse
 * - Triangle (Y): Reset REV Through Bore Encoder position
 * - D-Pad Up / Down: Increase / Decrease custom servo speed (+/- 0.1)
 * - D-Pad Left / Right: Decrease / Increase TransferConfig.TICKS_PER_REV (+/- 100)
 */
@TeleOp(name = "Transfer Subsystem Test", group = "Testing")
@SuppressWarnings("unused")
public class TransferTestOpMode extends LinearOpMode {

    private double customSpeed = Math.abs(TransferConfig.FORWARD_POWER);
    //boolean autoTransfer = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Multi-telemetry pushes logs to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);

        telemetry.addLine("=== Transfer Subsystem Test ===");
        telemetry.addLine("Cross / RT: Rotate Forward | Square / LT: Reverse");
        telemetry.addLine("Right/Left Bumpers: Advance 1 Slot (+/- 1/4 Rev)");
        telemetry.addLine("Circle: Stop Transfer | Triangle: Reset Encoder");
        telemetry.addLine("D-Pad Up/Down: Change Speed | D-Pad Left/Right: Adj Ticks/Rev");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Speed adjustment via D-Pad Up / Down
            if (gamepad1.dpadUpWasPressed()) {
                customSpeed = Math.min(1.0, customSpeed + 0.1);
            } else if (gamepad1.dpadDownWasPressed()) {
                customSpeed = Math.max(0.1, customSpeed - 0.1);
            }

            // TICKS_PER_REV adjustment via D-Pad Left / Right
            if (gamepad1.dpadLeftWasPressed()) {
                TransferConfig.TICKS_PER_REV = Math.max(100.0, TransferConfig.TICKS_PER_REV - 100.0);
            } else if (gamepad1.dpadRightWasPressed()) {
                TransferConfig.TICKS_PER_REV += 100.0;
            }

            // Servo Rotation Controls
            if (gamepad1.cross || gamepad1.right_trigger > 0.1) {
                transfer.setPower(-customSpeed);
            } else if (gamepad1.square || gamepad1.left_trigger > 0.1) {
                transfer.setPower(customSpeed);
            } else if (gamepad1.circle) {
                transfer.stop();
            }

            // Advance 1/4 full rotation (1 slot) via Right / Left Bumpers
            if (gamepad1.rightBumperWasPressed()) {
                transfer.Shoot1Ball();
            } else if (gamepad1.leftBumperWasPressed()) {
                transfer.Load1Ball();
            }

           /* if (gamepad1.optionsWasPressed() && gamepad1.circleWasPressed()) {
                autoTransfer = true;
            }

            if (autoTransfer) {
                if (transfer.isSlotOccupied(TransferSubsystem.Slot.BOTTOM)) {
                    transfer.advance1SlotForward();
                }
            }*/
            // Reset Through Bore Encoder Position via Triangle
            if (gamepad1.triangleWasPressed()) {
                transfer.resetEncoder();
            }

            // Update active subsystem state transitions (e.g. 1-slot advancement)
            transfer.update();

            // Telemetry Outputs: Servo State & Config
            telemetry.addLine("=== Transfer Servo ===");
            telemetry.addData("Servo Power", String.format(Locale.US, "%.2f", transfer.getPower()));
            telemetry.addData("Speed Setting", String.format(Locale.US, "%.2f", customSpeed));
            telemetry.addData("Is Rotating", transfer.isRotating() ? "YES" : "NO");

            // Telemetry Outputs: REV Through Bore Encoder
            telemetry.addLine("=== Through Bore Encoder ===");
            telemetry.addData("Ticks Per Rev", String.format(Locale.US, "%.0f (D-Pad Left/Right)", TransferConfig.TICKS_PER_REV));
            telemetry.addData("Raw Ticks", String.format(Locale.US, "%.0f", transfer.getRawTicks()));
            telemetry.addData("Rotations", String.format(Locale.US, "%.3f rev", transfer.getRotations()));
            telemetry.addData("Angle", String.format(Locale.US, "%.1f deg", transfer.getDegrees()));
            telemetry.addData("Velocity", String.format(Locale.US, "%.1f ticks/s (%.1f RPM)",
                    transfer.getVelocityTicksPerSec(), transfer.getVelocityRPM()));

            // Telemetry Outputs: Four Ball Slots (Top, Left, Right, Bottom)
            telemetry.addLine("=== Transfer Ball Occupancy ===");
            telemetry.addData("Ball Count", transfer.getBallCount() + " / 4");
            telemetry.addData("Transfer Status", transfer.isFull() ? "FULL" : (transfer.isEmpty() ? "EMPTY" : "PARTIAL"));

            // Slot 0: Top Slot
            boolean topPresent = transfer.isSlotOccupied(TransferSubsystem.SENSOR_TOP);
            BallColor topColor = transfer.detectSlotColor(TransferSubsystem.SENSOR_TOP);
            double topDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_TOP);
            NormalizedRGBA topRgba = transfer.getNormalizedColors(TransferSubsystem.SENSOR_TOP);
            String topDistStr = (topDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", topDist);
            telemetry.addData("Top Slot", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    topPresent ? "YES" : "NO ", topColor, topDistStr, topRgba.red, topRgba.green, topRgba.blue));

            // Slot 1: Left Slot
            boolean leftPresent = transfer.isSlotOccupied(TransferSubsystem.SENSOR_LEFT);
            BallColor leftColor = transfer.detectSlotColor(TransferSubsystem.SENSOR_LEFT);
            double leftDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_LEFT);
            NormalizedRGBA leftRgba = transfer.getNormalizedColors(TransferSubsystem.SENSOR_LEFT);
            String leftDistStr = (leftDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", leftDist);
            telemetry.addData("Left Slot", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    leftPresent ? "YES" : "NO ", leftColor, leftDistStr, leftRgba.red, leftRgba.green, leftRgba.blue));

            // Slot 2: Right Slot
            boolean rightPresent = transfer.isSlotOccupied(TransferSubsystem.SENSOR_RIGHT);
            BallColor rightColor = transfer.detectSlotColor(TransferSubsystem.SENSOR_RIGHT);
            double rightDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_RIGHT);
            NormalizedRGBA rightRgba = transfer.getNormalizedColors(TransferSubsystem.SENSOR_RIGHT);
            String rightDistStr = (rightDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", rightDist);
            telemetry.addData("Right Slot", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    rightPresent ? "YES" : "NO ", rightColor, rightDistStr, rightRgba.red, rightRgba.green, rightRgba.blue));

            // Slot 3: Bottom Slot (Combined status for Bottom Back and Bottom Front sensors)
            boolean bottomPresent = transfer.isSlotOccupied(TransferSubsystem.Slot.BOTTOM);
            BallColor bottomColor = transfer.detectSlotColor(TransferSubsystem.Slot.BOTTOM);
            telemetry.addData("Bottom Slot (Overall)", String.format(Locale.US, "Present: %s | Color: %s",
                    bottomPresent ? "YES" : "NO ", bottomColor));

            // Bottom Back Sensor
            boolean bbPresent = transfer.isSensorDetectingBall(TransferSubsystem.SENSOR_BOTTOM_BACK);
            BallColor bbColor = transfer.detectSensorColor(TransferSubsystem.SENSOR_BOTTOM_BACK);
            double bbDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_BACK);
            NormalizedRGBA bbRgba = transfer.getNormalizedColors(TransferSubsystem.SENSOR_BOTTOM_BACK);
            String bbDistStr = (bbDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", bbDist);
            telemetry.addData("  └ Bottom Back Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    bbPresent ? "YES" : "NO ", bbColor, bbDistStr, bbRgba.red, bbRgba.green, bbRgba.blue));

            // Bottom Front Sensor
            boolean bfPresent = transfer.isSensorDetectingBall(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            BallColor bfColor = transfer.detectSensorColor(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            double bfDist = transfer.getDistanceMm(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            NormalizedRGBA bfRgba = transfer.getNormalizedColors(TransferSubsystem.SENSOR_BOTTOM_FRONT);
            String bfDistStr = (bfDist == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", bfDist);
            telemetry.addData("  └ Bottom Front Sensor", String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | R:%.2f G:%.2f B:%.2f",
                    bfPresent ? "YES" : "NO ", bfColor, bfDistStr, bfRgba.red, bfRgba.green, bfRgba.blue));

            telemetry.addLine("=== Controls ===");
            telemetry.addLine("Cross/RT: Forward | Square/LT: Reverse | Circle: Stop");
            telemetry.addLine("Bumpers Right/Left: Advance 1 Slot Forward/Reverse (+/- 1/4 Rev)");
            telemetry.addLine("Triangle: Reset Encoder | D-Pad Up/Down: Change Speed | D-Pad Left/Right: Adj Ticks/Rev");

            telemetry.update();
        }

        transfer.stop();
    }
}
