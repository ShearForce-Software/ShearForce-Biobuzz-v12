package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo shooterServo;


    private static final double SHOOTER_SERVO_REST = 0.0;
    private static final double SHOOTER_SERVO_SHOOT = 1.0;

    // Tune these PIDF values for your specific flywheel setup (ticks/sec)
    // F is the feedforward constant (~ 32767 / max_ticks_per_sec)
    // Cache the active PIDF values to detect Dashboard changes
    private double lastP;
    private double lastI;
    private double lastD;
    private double lastF;
    public static final double TARGET_VELOCITY_TICKS = 2100.0;
    public static final double VELOCITY_TOLERANCE = 50.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        // Dual flywheels face opposite directions
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Seed initial PIDF gains from ShooterConfig
        lastP = ShooterConfig.P;
        lastI = ShooterConfig.I;
        lastD = ShooterConfig.D;
        lastF = ShooterConfig.F;

        PIDFCoefficients initialPidf = new PIDFCoefficients(lastP, lastI, lastD, lastF);

        // Configure both flywheels for closed-loop velocity control
        for (DcMotorEx motor : new DcMotorEx[]{shooterLeft, shooterRight}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, initialPidf);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coasting prevents gear wear
        }

        shooterServo.setPosition(SHOOTER_SERVO_REST);
    }

    /**
     * Call this inside your OpMode's main loop.
     * Checks if FTC Dashboard has modified any PIDF gains and updates the motors on the fly.
     */
    public void update() {
        if (lastP != ShooterConfig.P ||
                lastI != ShooterConfig.I ||
                lastD != ShooterConfig.D ||
                lastF != ShooterConfig.F) {

            lastP = ShooterConfig.P;
            lastI = ShooterConfig.I;
            lastD = ShooterConfig.D;
            lastF = ShooterConfig.F;

            PIDFCoefficients updatedPidf = new PIDFCoefficients(lastP, lastI, lastD, lastF);

            shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, updatedPidf);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, updatedPidf);
        }
    }

    // --- Hardware Control Methods ---

    public void startFlywheels() {
        shooterLeft.setVelocity(TARGET_VELOCITY_TICKS);
        shooterRight.setVelocity(TARGET_VELOCITY_TICKS);
    }

    public void setVelocity(double ticksPerSec) {
        shooterLeft.setVelocity(ticksPerSec);
        shooterRight.setVelocity(ticksPerSec);
    }

    public void stopFlywheels() {
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    public boolean isAtSpeed() {
        return Math.abs(shooterLeft.getVelocity() - TARGET_VELOCITY_TICKS) < VELOCITY_TOLERANCE &&
                Math.abs(shooterRight.getVelocity() - TARGET_VELOCITY_TICKS) < VELOCITY_TOLERANCE;
    }

    public void firePosition() {
        shooterServo.setPosition(SHOOTER_SERVO_SHOOT);
    }

    public void restPosition() {
        shooterServo.setPosition(SHOOTER_SERVO_REST);
    }

    public double getAverageVelocity() {
        return (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
    }

    public double getLeftVelocity() {
        return shooterLeft.getVelocity();
    }

    public double getRightVelocity() {
        return shooterRight.getVelocity();
    }

    // --- Ivy Command Generators ---

    /**
     * Complete firing sequence for use directly in Ivy sequential command lists:
     * Spools flywheels -> waits until at speed -> pushes game element -> resets servo.
     */
    public Command firePositionCommand() {
        return sequential(
                instant(this::startFlywheels),
                waitUntil(this::isAtSpeed),
                instant(this::firePosition),
                waitMs(250), // Adjust delay based on how long the physical pusher takes
                instant(this::restPosition)
        );
    }

    /**
     * Standalone command to spin up flywheels.
     */
    public Command startFlywheelsCommand() {
        return instant(this::startFlywheels);
    }

    /**
     * Standalone command to power down flywheels.
     */
    public Command stopFlywheelsCommand() {
        return instant(this::stopFlywheels);
    }

}
