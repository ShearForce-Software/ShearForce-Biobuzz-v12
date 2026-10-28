package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.TransferConfig;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.TransferSubsystem.BallColor;

import java.util.Locale;

/**
 * Interactive Test and Tuning OpMode for the TransferSubsystem.
 *
 * Gamepad 1 Controls:
 * - Cross (A) / Right Trigger: Rotate transfer forward
 * - Square (X) / Left Trigger: Rotate transfer in reverse
 * - Circle (B): Stop transfer rotation
 * - Triangle (Y): Reset REV Through Bore Encoder position
 * - D-Pad Up / Down: Increase / Decrease custom servo speed (+/- 0.1)
 */
@TeleOp(name = "Transfer Subsystem Test", group = "Testing")
@SuppressWarnings("unused")
public class TransferTestOpMode extends LinearOpMode {

    private TransferSubsystem transfer;

    private double customSpeed = TransferConfig.FORWARD_POWER;

    // Edge detection for button toggles
    private boolean lastTriangle = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean rotateModeOn = false;
    private double targetTicks = 0;
    private double rotation1 = TransferConfig.TICKS_PER_REV;


    @Override
    public void runOpMode() throws InterruptedException {
        // Multi-telemetry pushes logs to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        transfer = new TransferSubsystem(hardwareMap);

        telemetry.addLine("=== Transfer Subsystem Test ===");
        telemetry.addLine("Cross / RT: Rotate Forward");
        telemetry.addLine("Square / LT: Rotate Reverse");
        telemetry.addLine("Circle: Stop Transfer");
        telemetry.addLine("Triangle: Reset Encoder");
        telemetry.addLine("D-Pad Up/Down: Adjust Speed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Speed adjustment via D-Pad Up / Down
            boolean currentDpadUp = gamepad1.dpad_up;
            if (currentDpadUp && !lastDpadUp) {
                customSpeed = Math.min(1.0, customSpeed + 0.1);
            }
            lastDpadUp = currentDpadUp;

            boolean currentDpadDown = gamepad1.dpad_down;
            if (currentDpadDown && !lastDpadDown) {
                customSpeed = Math.max(0.1, customSpeed - 0.1);
            }
            lastDpadDown = currentDpadDown;

            // Servo Rotation Controls
            if (gamepad1.cross || gamepad1.right_trigger > 0.1) {
                transfer.setPower(customSpeed);
            } else if (gamepad1.square || gamepad1.left_trigger > 0.1) {
                transfer.setPower(-customSpeed);
            } else if (gamepad1.circle) {
                transfer.stop();
            }

            //move 1 rotation
            if(gamepad1.dpadRightWasPressed()){
                transfer.setPower(customSpeed);
                double encoderCurrentTicks = transfer.getRawTicks();
                targetTicks = encoderCurrentTicks + rotation1;
                rotateModeOn = true;
            }
            if (rotateModeOn){
                if (transfer.getRawTicks() >= targetTicks){
                    transfer.stop();
                    rotateModeOn = false;
                }
            }
            // Reset Through Bore Encoder Position
            boolean currentTriangle = gamepad1.triangle;
            if (currentTriangle && !lastTriangle) {
                transfer.resetEncoder();
            }
            lastTriangle = currentTriangle;

            // Telemetry Outputs: Servo State & Config
            telemetry.addLine("=== Transfer Servo ===");
            telemetry.addData("Servo Power", String.format(Locale.US, "%.2f", transfer.getPower()));
            telemetry.addData("Speed Setting", String.format(Locale.US, "%.2f", customSpeed));
            telemetry.addData("Is Rotating", transfer.isRotating() ? "YES" : "NO");

            // Telemetry Outputs: REV Through Bore Encoder
            telemetry.addLine("=== Through Bore Encoder ===");
            telemetry.addData("Raw Ticks", String.format(Locale.US, "%.0f", transfer.getRawTicks()));
            telemetry.addData("Rotations", String.format(Locale.US, "%.3f rev", transfer.getRotations()));
            telemetry.addData("Angle", String.format(Locale.US, "%.1f deg", transfer.getDegrees()));
            telemetry.addData("Velocity", String.format(Locale.US, "%.1f ticks/s (%.1f RPM)",
                    transfer.getVelocityTicksPerSec(), transfer.getVelocityRPM()));

            // Telemetry Outputs: Four Color / Distance Sensors
            telemetry.addLine("=== Transfer Ball Occupancy ===");
            telemetry.addData("Ball Count", transfer.getBallCount() + " / 4");
            telemetry.addData("Transfer Status", transfer.isFull() ? "FULL" : (transfer.isEmpty() ? "EMPTY" : "PARTIAL"));

            for (int i = 0; i < 4; i++) {
                boolean present = transfer.isBallPresent(i);
                BallColor color = transfer.detectBallColor(i);
                double distanceMm = transfer.getDistanceMm(i);
                NormalizedRGBA rgba = transfer.getNormalizedColors(i);

                String distanceStr = (distanceMm == Double.MAX_VALUE) ? "N/A" : String.format(Locale.US, "%.1f mm", distanceMm);
                String rgbaStr = String.format(Locale.US, "R:%.2f G:%.2f B:%.2f", rgba.red, rgba.green, rgba.blue);

                telemetry.addData(String.format(Locale.US, "Slot %d", i + 1),
                        String.format(Locale.US, "Present: %s | Color: %s | Dist: %s | %s",
                                present ? "YES" : "NO ", color, distanceStr, rgbaStr));
            }

            telemetry.addLine("=== Controls ===");
            telemetry.addLine("Cross/RT: Forward | Square/LT: Reverse | Circle: Stop");
            telemetry.addLine("Triangle: Reset Encoder | D-Pad Up/Down: Change Speed");

            telemetry.update();
        }

        transfer.stop();
    }
}
