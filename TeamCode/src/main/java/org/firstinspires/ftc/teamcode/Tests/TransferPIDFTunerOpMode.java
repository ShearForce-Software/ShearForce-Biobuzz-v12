package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferConfig;
import org.firstinspires.ftc.teamcode.Biobuzz.subsystems.TransferSubsystem;

/**
 * Interactive Slot-Based PIDF Tuner OpMode for the TransferSubsystem.
 *
 * Allows tuning the CRServo position & velocity PIDF gains live via PS4/Gamepad controller
 * to ensure the transfer system rotates by exact slot/tick increments and stops accurately in time.
 *
 * TUNING WORKFLOW GUIDELINES FOR ACCURATE & CONSISTENT SLOT MOVEMENT:
 * 1. Start with POSITION_I = 0, POSITION_D = 0, and POSITION_F = 0.
 * 2. Increase POSITION_P until the transfer disk moves quickly toward the target slot.
 * 3. Increase POSITION_D to damp out overshoot/bouncing as the slot settles (uses direct encoder velocity).
 * 4. If the disk stops 15-30 ticks short of the target due to CRServo static friction,
 *    add a small POSITION_F feedforward (0.03 to 0.06) to overcome breakaway static friction.
 * 5. Verify settling with 0, 1, 2, 3, and 4 balls loaded to ensure consistent slot indexing under variable mass.
 *
 * PS4 / Gamepad 1 Controls:
 * - Cross (A): Execute PIDF Rotation FORWARD by selected slot increment
 * - Triangle (Y): Execute PIDF Rotation REVERSE by selected slot increment
 * - Circle (B): Emergency Stop Servo
 * - Left/Right Bumpers: Decrease / Increase Slot Increment (0.25, 0.5, 1.0, 2.0, 3.0, 4.0 slots)
 * - Square (X): Cycle Active PIDF Parameter to Tune (P, I, D, F, Max Decel)
 * - D-Pad Up / Down: Increase / Decrease Active Parameter by Step Size
 * - D-Pad Left / Right: Cycle Adjustment Step Size (1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001)
 * - Left Stick Button (L3): Reset REV Through Bore Encoder Position Count to 0
 */
@Utility(name = "Transfer PIDF Tuner", description = "Slot Increment & PIDF Tuning for Transfer Subsystem")
@SuppressWarnings("unused")
public class TransferPIDFTunerOpMode extends LinearOpMode {

    public enum PIDFParameter {
        POSITION_P("POSITION_P (Proportional Gain)"),
        POSITION_I("POSITION_I (Integral Gain)"),
        POSITION_D("POSITION_D (Derivative Gain)"),
        POSITION_F("POSITION_F (Static Friction Feedforward)"),
        VELOCITY_F("VELOCITY_F (Feedforward Gain)"),
        MAX_DECEL("MAX_DECEL (Max Decel Ticks/s²)");

        public final String displayName;

        PIDFParameter(String displayName) {
            this.displayName = displayName;
        }
    }

    // Slot increment options (1 slot = 2048 ticks = 90 degrees)
    private final double[] slotIncrementOptions = {0.25, 0.5, 1.0, 2.0, 3.0, 4.0};
    private int slotIncrementIndex = 2; // Default = 1.0 slot

    // Step sizes for tuning PIDF parameters
    private final double[] stepSizes = {1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 3; // Default step size = 0.001

    // Active PIDF parameter selection
    private int activeParamIndex = 0;
    private final PIDFParameter[] parameters = PIDFParameter.values();

    @Override
    public void runOpMode() throws InterruptedException {
        // Wrap Driver Station and Dashboard telemetry together
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Temporarily disable auto-transfer logic during PIDF tuning to isolate pure slot movements
        TransferConfig.AUTO_TRANSFER = false;

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, intake);

        telemetry.addLine("=== Transfer Slot PIDF Tuner ===");
        telemetry.addLine("Press PLAY to start interactive slot & PIDF tuning.");
        telemetry.addLine("-------------------------------------------------");
        telemetry.addLine("Cross (A): Rotate FORWARD by Slot Increment");
        telemetry.addLine("Triangle (Y): Rotate REVERSE by Slot Increment");
        telemetry.addLine("Circle (B): Emergency Stop Servo");
        telemetry.addLine("LB / RB: Decrease / Increase Slot Increment");
        telemetry.addLine("Square (X): Cycle Active PIDF Parameter");
        telemetry.addLine("D-Pad Up/Down: Adjust Parameter (+/-)");
        telemetry.addLine("D-Pad Left/Right: Cycle Step Size");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Slot Increment Selection (Bumpers) ---
            if (gamepad1.leftBumperWasPressed()) {
                slotIncrementIndex = Math.max(0, slotIncrementIndex - 1);
            } else if (gamepad1.rightBumperWasPressed()) {
                slotIncrementIndex = Math.min(slotIncrementOptions.length - 1, slotIncrementIndex + 1);
            }

            double currentSlotIncrement = slotIncrementOptions[slotIncrementIndex];

            // --- Trigger PIDF Slot Movement Commands ---

            // Cross (A): Rotate Forward by selected slot increment
            if (gamepad1.crossWasPressed() || gamepad1.aWasPressed()) {
                transfer.rotateSlotsPIDF(currentSlotIncrement);
            }

            // Triangle (Y): Rotate Reverse by selected slot increment
            if (gamepad1.triangleWasPressed() || gamepad1.yWasPressed()) {
                transfer.rotateSlotsPIDF(-currentSlotIncrement);
            }

            // Circle (B): Emergency Stop
            if (gamepad1.circleWasPressed() || gamepad1.bWasPressed()) {
                transfer.stop();
            }

            // Left Stick Button (L3): Reset Encoder Count
            if (gamepad1.left_stick_button) {
                transfer.resetEncoder();
            }

            // --- PIDF Parameter Selection & Tuning ---

            // Square (X): Cycle active PIDF parameter
            if (gamepad1.squareWasPressed() || gamepad1.xWasPressed()) {
                activeParamIndex = (activeParamIndex + 1) % parameters.length;
            }

            // D-Pad Left / Right: Cycle adjustment step size
            if (gamepad1.dpadLeftWasPressed()) {
                stepIndex = Math.max(0, stepIndex - 1);
            } else if (gamepad1.dpadRightWasPressed()) {
                stepIndex = Math.min(stepSizes.length - 1, stepIndex + 1);
            }

            double step = stepSizes[stepIndex];
            PIDFParameter activeParam = parameters[activeParamIndex];

            // D-Pad Up / Down: Adjust active selected PIDF parameter
            if (gamepad1.dpadUpWasPressed()) {
                adjustPIDFParameter(activeParam, step);
            } else if (gamepad1.dpadDownWasPressed()) {
                adjustPIDFParameter(activeParam, -step);
            }

            // Execute subsystem update loop (runs PIDF calculations + sensor queries)
            transfer.update();

            // --- Driver Station Telemetry Display ---

            double targetTicks = transfer.getTargetPositionTicks();
            double currentTicks = transfer.getRawTicks();
            double posError = transfer.getPositionError();
            double errorDegrees = (posError / TransferConfig.TICKS_PER_REV) * 360.0;
            double targetTicksIncrement = currentSlotIncrement * TransferConfig.TICKS_PER_SLOT;

            telemetry.addLine("=== Transfer Slot PIDF Tuner ===");
            telemetry.addData("State", transfer.getControlState());
            telemetry.addData(">>> Active Parameter <<<", "%s [Value: %.6f]", activeParam.displayName, getPIDFParameterValue(activeParam));
            telemetry.addData("Step Size (D-Pad L/R)", "%.5f", step);

            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine("=== Slot Increment & Position Target ===");
            telemetry.addData("Slot Increment (LB/RB)", "%.2f slots (%.0f ticks / %.1f deg)",
                    currentSlotIncrement, targetTicksIncrement, currentSlotIncrement * 90.0);
            telemetry.addData("Target / Actual Position", "Target: %.0f | Actual: %.0f ticks", targetTicks, currentTicks);
            telemetry.addData("Position Error", "%.1f ticks (%.2f deg)", posError, errorDegrees);

            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine("=== Servo & Encoder Status ===");
            telemetry.addData("Servo Power", "%.3f", transfer.getPower());
            telemetry.addData("Velocity", "%.1f ticks/s (%.1f RPM)", transfer.getVelocityTicksPerSec(), transfer.getVelocityRPM());
            telemetry.addData("Encoder Rotations", "%.3f rev (%.1f deg)", transfer.getRotations(), transfer.getDegrees());

            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine("=== Individual PIDF Values ===");
            telemetry.addData("POSITION_P", "%.6f %s", TransferConfig.POSITION_P, activeParam == PIDFParameter.POSITION_P ? " <-- EDITING" : "");
            telemetry.addData("POSITION_I", "%.6f %s", TransferConfig.POSITION_I, activeParam == PIDFParameter.POSITION_I ? " <-- EDITING" : "");
            telemetry.addData("POSITION_D", "%.6f %s", TransferConfig.POSITION_D, activeParam == PIDFParameter.POSITION_D ? " <-- EDITING" : "");
            telemetry.addData("POSITION_F", "%.6f %s", TransferConfig.POSITION_F, activeParam == PIDFParameter.POSITION_F ? " <-- EDITING" : "");
            telemetry.addData("VELOCITY_F", "%.6f %s", TransferConfig.VELOCITY_F, activeParam == PIDFParameter.VELOCITY_F ? " <-- EDITING" : "");
            telemetry.addData("MAX_DECEL", "%.1f ticks/s² %s", TransferConfig.MAX_DECEL_TICKS_PER_SEC2, activeParam == PIDFParameter.MAX_DECEL ? " <-- EDITING" : "");

            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine("=== Slot Occupancy ===");
            telemetry.addData("Ball Count / Status", "%d / 4 [%s]", transfer.getBallCount(), transfer.isFull() ? "FULL" : (transfer.isEmpty() ? "EMPTY" : "PARTIAL"));
            telemetry.addData("Nearest Empty Slot", transfer.findNearestEmptySlot());

            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine("=== PS4 Controller Controls ===");
            telemetry.addLine("Cross (A): Rotate Forward by Slot Increment");
            telemetry.addLine("Triangle (Y): Rotate Reverse by Slot Increment");
            telemetry.addLine("Circle (B): Emergency Stop Servo");
            telemetry.addLine("LB / RB: Decrease / Increase Slot Increment");
            telemetry.addLine("Square (X): Cycle Active PIDF Parameter");
            telemetry.addLine("D-Pad Up/Down: Increase/Decrease Active Parameter");
            telemetry.addLine("D-Pad Left/Right: Decrease/Increase Step Size");
            telemetry.addLine("L3 Button: Reset Encoder Count to 0");

            telemetry.update();
        }

        transfer.stop();
    }

    private void adjustPIDFParameter(PIDFParameter param, double delta) {
        switch (param) {
            case POSITION_P:
                TransferConfig.POSITION_P = Math.max(0.0, TransferConfig.POSITION_P + delta);
                break;
            case POSITION_I:
                TransferConfig.POSITION_I = Math.max(0.0, TransferConfig.POSITION_I + delta);
                break;
            case POSITION_D:
                TransferConfig.POSITION_D = Math.max(0.0, TransferConfig.POSITION_D + delta);
                break;
            case POSITION_F:
                TransferConfig.POSITION_F = Math.max(0.0, TransferConfig.POSITION_F + delta);
                break;
            case VELOCITY_F:
                TransferConfig.VELOCITY_F = Math.max(0.0, TransferConfig.VELOCITY_F + delta);
                break;
            case MAX_DECEL:
                TransferConfig.MAX_DECEL_TICKS_PER_SEC2 = Math.max(100.0, TransferConfig.MAX_DECEL_TICKS_PER_SEC2 + delta);
                break;
        }
    }

    private double getPIDFParameterValue(PIDFParameter param) {
        switch (param) {
            case POSITION_P: return TransferConfig.POSITION_P;
            case POSITION_I: return TransferConfig.POSITION_I;
            case POSITION_D: return TransferConfig.POSITION_D;
            case POSITION_F: return TransferConfig.POSITION_F;
            case VELOCITY_F: return TransferConfig.VELOCITY_F;
            case MAX_DECEL: return TransferConfig.MAX_DECEL_TICKS_PER_SEC2;
            default: return 0.0;
        }
    }
}
