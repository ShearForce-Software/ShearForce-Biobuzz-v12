package org.firstinspires.ftc.teamcode.summerPedro.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterConfig;
import org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterSubsystem;

/*
This is an OpMode demonstrating both live PIDF updating and live graphing using TelemetryPacket

How to View the Graph in the Browser
1. Connect your laptop to the robot's Wi-Fi network.
2. Open Dashboard: Navigate to http://192.168.43.1:8080/dash in Chrome
3. Open Graph Panel:
	• In the top-right menu, ensure the Graph view / checkbox is enabled.
4. Select Plot Traces:
	• Under the graph pane, you will see a list of plotted keys
	  (Target Velocity, Left Velocity, Right Velocity, Average Velocity).

	• Check each box to overlay them onto the same real-time graph.
5. Adjust Parameters:
	• Open the ShooterConfig dropdown on the dashboard panel.
	• Edit P, I, D, F, or TARGET_VELOCITY_TICKS.
	• Watch the step response in real time to check for overshoot, oscillation, or steady-state error.
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterTuningOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Wrap Driver Station telemetry and FTC Dashboard telemetry together
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize Dashboard and Subsystem
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("Ready to start. Open http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        // Spin up flywheels to target speed
        shooter.startFlywheels();

        while (opModeIsActive()) {
            // Check for on-the-fly PIDF/target changes from dashboard
            shooter.update();

            // Broadcast numerical values to BOTH Driver Station and the Dashboard graph
            telemetry.addData("Target Velocity", ShooterConfig.TARGET_VELOCITY_TICKS);
            telemetry.addData("Left Velocity", shooter.getLeftVelocity());
            telemetry.addData("Right Velocity", shooter.getRightVelocity());
            telemetry.addData("Average Velocity", shooter.getAverageVelocity());
            telemetry.addData("At Speed", shooter.isAtSpeed());

            telemetry.update();
        }

        shooter.stopFlywheels();
    }
}