package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.IntakeSubsystem;

/**
 * Interactive Test OpMode for the IntakeSubsystem.
 *
 * Gamepad 1 Controls:
 * - Cross (A) / Right Trigger: Turn on intake (spin forward to intake balls)
 * - Square (X) / Left Trigger: Reverse intake (spin backward to eject balls)
 * - Circle (B): Turn off intake
 */
@TeleOp(name = "Intake Subsystem Test", group = "Testing")
@SuppressWarnings("unused")
public class IntakeTestOpMode extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        // Multi-telemetry pushes logs to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        telemetry.addLine("=== Intake Subsystem Test ===");
        telemetry.addLine("Cross / RT: Intake Forward");
        telemetry.addLine("Square / LT: Eject / Reverse");
        telemetry.addLine("Circle: Turn Off");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad 1 Controls
            if (gamepad1.cross || gamepad1.right_trigger > 0.1) {
                intakeSubsystem.turnOn();
            } else if (gamepad1.square || gamepad1.left_trigger > 0.1) {
                intakeSubsystem.reverse();
            } else if (gamepad1.circle) {
                intakeSubsystem.turnOff();
            }

            // Status Telemetry
            telemetry.addLine("=== Intake Status ===");
            //telemetry.addData("Capacity Sensor Triggered", intakeSubsystem.isMaxCapacityReached() ? "YES (FULL)" : "NO");

            telemetry.addLine("=== Controls ===");
            telemetry.addLine("Cross / RT: Turn On (Forward)");
            telemetry.addLine("Square / LT: Reverse (Eject)");
            telemetry.addLine("Circle: Turn Off");

            telemetry.update();
        }

        intakeSubsystem.turnOff();
    }
}
