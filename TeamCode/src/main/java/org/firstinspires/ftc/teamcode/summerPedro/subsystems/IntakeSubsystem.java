package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor; // Mock capacity sensor

public class IntakeSubsystem {
    private final DcMotorEx intakeMotor;
    private final TouchSensor capacitySensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        capacitySensor = hardwareMap.get(TouchSensor.class, "capacitySensor");
    }

    // --- Hardware Control Methods ---

    public void turnOn() {
        intakeMotor.setPower(1.0);
    }

    public void turnOff() {
        intakeMotor.setPower(0.0);
    }

    public void reverse() {
        intakeMotor.setPower(-1.0);
    }

    public boolean isMaxCapacityReached() {
        return capacitySensor.isPressed();
    }

    // --- Ivy Command Generators ---

    /**
     * Command to turn on the intake.
     */
    public Command turnOnCommand() {
        return instant(this::turnOn);
    }

    /**
     * Command to turn off the intake.
     */
    public Command turnOffCommand() {
        return instant(this::turnOff);
    }

    /**
     * Command to reverse the intake.
     */
    public Command reverseCommand() {
        return instant(this::reverse);
    }

    /**
     * Complete intake sequence: turns on intake, waits until max capacity sensor is triggered, then turns off intake.
     */
    public Command intakeUntilFullCommand() {
        return sequential(
                instant(this::turnOn),
                waitUntil(this::isMaxCapacityReached),
                instant(this::turnOff)
        );
    }
}
