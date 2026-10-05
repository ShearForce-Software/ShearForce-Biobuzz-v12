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

    // Cache active PIDF & Target values to detect Dashboard changes
    private double lastP;
    private double lastI;
    private double lastD;
    private double lastF;
    private double lastTargetVelocity;
    private boolean isRunning = false;

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
        lastTargetVelocity = ShooterConfig.TARGET_VELOCITY_TICKS;

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
     * Checks if FTC Dashboard has modified any PIDF gains or target velocity and updates the motors on the fly.
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

        if (lastTargetVelocity != ShooterConfig.TARGET_VELOCITY_TICKS) {
            lastTargetVelocity = ShooterConfig.TARGET_VELOCITY_TICKS;
            if (isRunning) {
                shooterLeft.setVelocity(ShooterConfig.TARGET_VELOCITY_TICKS);
                shooterRight.setVelocity(ShooterConfig.TARGET_VELOCITY_TICKS);
            }
        }
    }

    // --- Hardware Control Methods ---

    public void startFlywheels() {
        isRunning = true;
        shooterLeft.setVelocity(ShooterConfig.TARGET_VELOCITY_TICKS);
        shooterRight.setVelocity(ShooterConfig.TARGET_VELOCITY_TICKS);
    }

    public void setVelocity(double ticksPerSec) {
        ShooterConfig.TARGET_VELOCITY_TICKS = ticksPerSec;
        lastTargetVelocity = ticksPerSec;
        isRunning = ticksPerSec != 0;
        shooterLeft.setVelocity(ticksPerSec);
        shooterRight.setVelocity(ticksPerSec);
    }

    public void stopFlywheels() {
        isRunning = false;
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isAtSpeed() {
        return Math.abs(shooterLeft.getVelocity() - ShooterConfig.TARGET_VELOCITY_TICKS) < ShooterConfig.VELOCITY_TOLERANCE &&
                Math.abs(shooterRight.getVelocity() - ShooterConfig.TARGET_VELOCITY_TICKS) < ShooterConfig.VELOCITY_TOLERANCE;
    }

    public void firePosition() {
        shooterServo.setPosition(SHOOTER_SERVO_SHOOT);
    }

    public void restPosition() {
        shooterServo.setPosition(SHOOTER_SERVO_REST);
    }

    public double getTargetVelocity() {
        return ShooterConfig.TARGET_VELOCITY_TICKS;
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

    public Command firePositionCommand() {
        return sequential(
                instant(this::startFlywheels),
                waitUntil(this::isAtSpeed),
                instant(this::firePosition),
                waitMs(250),
                instant(this::restPosition)
        );
    }

    public Command startFlywheelsCommand() {
        return instant(this::startFlywheels);
    }

    public Command stopFlywheelsCommand() {
        return instant(this::stopFlywheels);
    }
}
