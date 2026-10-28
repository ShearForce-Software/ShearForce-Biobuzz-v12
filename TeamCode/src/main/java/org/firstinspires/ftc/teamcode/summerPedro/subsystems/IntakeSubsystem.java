package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor; // Mock capacity sensor

public class IntakeSubsystem {
    private final DcMotorEx intakeMotor;
    //private final TouchSensor capacitySensor;

    // Default target velocity in encoder ticks per second
    public double targetVelocityTicks = 1110.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        //capacitySensor = hardwareMap.get(TouchSensor.class, "capacitySensor");
    }

    // --- Hardware Control Methods ---

    public void turnOn() {
        intakeMotor.setVelocity(targetVelocityTicks);
    }

    public void turnOn(double ticksPerSecond) {
        intakeMotor.setVelocity(ticksPerSecond);
    }

    public void turnOff() {
        intakeMotor.setPower(0.0);
    }

    public void reverse() {
        intakeMotor.setVelocity(-targetVelocityTicks);
    }

    public void reverse(double ticksPerSecond) {
        intakeMotor.setVelocity(-Math.abs(ticksPerSecond));
    }

    public void setVelocity(double ticksPerSecond) {
        intakeMotor.setVelocity(ticksPerSecond);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    /*public boolean isMaxCapacityReached() {
        return capacitySensor.isPressed();
    }*/

    // --- Ivy Command Generators ---

    /**
     * Command to turn on the intake at default velocity.
     */
    public Command turnOnCommand() {
        return instant(this::turnOn);
    }

    /**
     * Command to turn on the intake at specified velocity in ticks/sec.
     */
    public Command turnOnCommand(double ticksPerSecond) {
        return instant(() -> turnOn(ticksPerSecond));
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

    public Command reverseCommand(double ticksPerSecond) {
        return instant(() -> reverse(ticksPerSecond));
    }

    /**
     * Complete intake sequence: turns on intake, waits until max capacity sensor is triggered, then turns off intake.
     */
    /*public Command intakeUntilFullCommand() {
        return sequential(
                instant(this::turnOn),
                waitUntil(this::isMaxCapacityReached),
                instant(this::turnOff)
        );
    }*/
    }

    public Command intakeUntilFullCommand(double ticksPerSecond) {
        return sequential(
                instant(() -> turnOn(ticksPerSecond)),
                waitUntil(this::isMaxCapacityReached),
                instant(this::turnOff)
        );
    }
}
