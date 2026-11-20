package org.firstinspires.ftc.teamcode.Biobuzz.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for managing the Intake mechanism.
 * Features:
 * - Intake motor velocity control
 * - Depollinator servo position control (Up / Down)
 * Note: Intake ramp color/distance sensors are managed centrally in TransferSubsystem.
 */
public class IntakeSubsystem {

    private final DcMotorEx intakeMotor;
    private final Servo depollinatorServo;

    // Default target velocity in encoder ticks per second
    public double targetVelocityTicks = IntakeConfig.DEFAULT_INTAKE_VELOCITY;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConfig.INTAKE_MOTOR_NAME);
        if (IntakeConfig.INTAKE_MOTOR_REVERSED) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        depollinatorServo = hardwareMap.get(Servo.class, IntakeConfig.DEPOLLINATOR_SERVO_NAME);

        // Default depollinator servo to UP position
        depollinatorUp();
    }

    // --- Hardware Motor Control Methods ---

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

    // --- Depollinator Servo Methods ---

    /**
     * Moves the depollinator servo to the UP position defined in IntakeConfig.
     */
    public void depollinatorUp() {
        depollinatorServo.setPosition(IntakeConfig.DEPOLLINATOR_SERVO_UP);
    }

    /**
     * Moves the depollinator servo to the DOWN position defined in IntakeConfig.
     */
    public void depollinatorDown() {
        depollinatorServo.setPosition(IntakeConfig.DEPOLLINATOR_SERVO_DOWN);
    }

    /**
     * Sets a custom raw position (0.0 to 1.0) to the depollinator servo.
     */
    public void setDepollinatorPosition(double position) {
        depollinatorServo.setPosition(position);
    }

    /**
     * Returns the current targeted position of the depollinator servo.
     */
    public double getDepollinatorPosition() {
        return depollinatorServo.getPosition();
    }

    /**
     * Returns true if the depollinator servo is currently closer to the configured UP position.
     */
    public boolean isDepollinatorUp() {
        return Math.abs(getDepollinatorPosition() - IntakeConfig.DEPOLLINATOR_SERVO_UP) < 0.1;
    }

    /**
     * Returns true if the depollinator servo is currently closer to the configured DOWN position.
     */
    public boolean isDepollinatorDown() {
        return Math.abs(getDepollinatorPosition() - IntakeConfig.DEPOLLINATOR_SERVO_DOWN) < 0.1;
    }

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
     * Command to reverse the intake at default velocity.
     */
    public Command reverseCommand() {
        return instant(this::reverse);
    }

    /**
     * Command to reverse the intake at specified velocity in ticks/sec.
     */
    public Command reverseCommand(double ticksPerSecond) {
        return instant(() -> reverse(ticksPerSecond));
    }

    /**
     * Command to move the depollinator servo to UP position.
     */
    public Command depollinatorUpCommand() {
        return instant(this::depollinatorUp);
    }

    /**
     * Command to move the depollinator servo to DOWN position.
     */
    public Command depollinatorDownCommand() {
        return instant(this::depollinatorDown);
    }
}
