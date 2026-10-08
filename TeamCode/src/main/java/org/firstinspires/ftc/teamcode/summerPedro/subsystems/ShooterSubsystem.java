package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.teamcode.summerPedro.subsystems.ShooterConfig.SHOOTER_SERVO_REST;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo shooterServo;

    // 1750 RPM = 816.7 ticks/sec for goBilda 1:1 Yellow Jacket motors (28 CPR)
    public double target_velocity_ticks = 816.7;

    // Cache active PIDF & Target values to detect Dashboard changes
    private double lastP;
    private double lastI;
    private double lastD;
    private double lastF;
    private double lastTargetVelocity;
    private boolean isRunning = false;

    // Timing tracking for speed recovery and 0-to-speed startup
    private final ElapsedTime speedTimer = new ElapsedTime();
    private final ElapsedTime startupTimer = new ElapsedTime();
    private boolean wasAtSpeed = false;
    private boolean isStartingUp = false;
    private double lastTimeToSpeed = -1.0; // In seconds (-1 if no measurement yet)
    private double lastStartupTime = -1.0;  // In seconds (-1 if no measurement yet)

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
        lastTargetVelocity = target_velocity_ticks;

        PIDFCoefficients initialPidf = new PIDFCoefficients(lastP, lastI, lastD, lastF);

        // Configure both flywheels for closed-loop velocity control
        for (DcMotorEx motor : new DcMotorEx[]{shooterLeft, shooterRight}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, initialPidf);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coasting prevents gear wear
        }

        shooterServo.setPosition(ShooterConfig.SHOOTER_SERVO_REST);
    }

    /**
     * Call this inside your OpMode's main loop.
     * Checks if FTC Dashboard has modified any PIDF gains or target velocity and updates the motors on the fly.
     * Also tracks time to reach speed and startup time.
     */
    public void update() {
        // if dashboard has adjusted the PIDF at all
        if (lastP != ShooterConfig.P ||
                lastI != ShooterConfig.I ||
                lastD != ShooterConfig.D ||
                lastF != ShooterConfig.F) {

            // 🔍 This will print to Android Studio's Logcat panel instantly when a packet lands
            android.util.Log.d("SHOOTER_DEBUG", "Dashboard Packet Received! New P: " + ShooterConfig.P + ", New F: " + ShooterConfig.F);

            lastP = ShooterConfig.P;
            lastI = ShooterConfig.I;
            lastD = ShooterConfig.D;
            lastF = ShooterConfig.F;

            PIDFCoefficients updatedPidf = new PIDFCoefficients(lastP, lastI, lastD, lastF);

            shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, updatedPidf);
            shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, updatedPidf);

            // Re-apply velocity so the REV Hub active velocity controller uses the updated PIDF gains
            if (isRunning) {
                shooterLeft.setVelocity(target_velocity_ticks);
                shooterRight.setVelocity(target_velocity_ticks);
            }
        }

        if (lastTargetVelocity != target_velocity_ticks) {
            lastTargetVelocity = target_velocity_ticks;
            if (isRunning) {
                speedTimer.reset();
                shooterLeft.setVelocity(target_velocity_ticks);
                shooterRight.setVelocity(target_velocity_ticks);
            }
        }

        // Timing logic: track time to reach target speed and startup time from 0 velocity
        boolean currentlyAtSpeed = isAtSpeed();

        if (!wasAtSpeed && currentlyAtSpeed) {
            // Just reached target speed!
            lastTimeToSpeed = speedTimer.seconds();

            if (isStartingUp) {
                lastStartupTime = startupTimer.seconds();
                isStartingUp = false;
            }
        } else if (wasAtSpeed && !currentlyAtSpeed) {
            // Dropped below target speed threshold; reset timer to measure recovery duration
            speedTimer.reset();
        }

        wasAtSpeed = currentlyAtSpeed;
    }

    // --- Hardware Control Methods ---

    public void startFlywheels() {
        if (!isRunning) {
            isStartingUp = true;
            startupTimer.reset();
            speedTimer.reset();
            wasAtSpeed = false;
        }
        isRunning = true;
        shooterLeft.setVelocity(target_velocity_ticks);
        shooterRight.setVelocity(target_velocity_ticks);
    }

    public void setVelocity(double ticksPerSec) {
        target_velocity_ticks = ticksPerSec;
        lastTargetVelocity = ticksPerSec;

        if (ticksPerSec > 0 && !isRunning) {
            isStartingUp = true;
            startupTimer.reset();
            speedTimer.reset();
            wasAtSpeed = false;
        } else if (ticksPerSec > 0) {
            speedTimer.reset();
            wasAtSpeed = false;
        }

        isRunning = ticksPerSec != 0;
        shooterLeft.setVelocity(ticksPerSec);
        shooterRight.setVelocity(ticksPerSec);
    }

    public void stopFlywheels() {
        isRunning = false;
        isStartingUp = false;
        wasAtSpeed = false;
        shooterLeft.setVelocity(0);
        shooterRight.setVelocity(0);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isAtSpeed() {
        return Math.abs(shooterLeft.getVelocity() - target_velocity_ticks) < ShooterConfig.VELOCITY_TOLERANCE &&
                Math.abs(shooterRight.getVelocity() - target_velocity_ticks) < ShooterConfig.VELOCITY_TOLERANCE;
    }

    public void firePosition() {
        shooterServo.setPosition(ShooterConfig.SHOOTER_SERVO_SHOOT);
    }

    public void restPosition() {
        shooterServo.setPosition(ShooterConfig.SHOOTER_SERVO_REST);
    }

    public double getTargetVelocity() {
        return target_velocity_ticks;
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

    public double getLastTimeToSpeed() {
        return lastTimeToSpeed;
    }

    public double getLastStartupTime() {
        return lastStartupTime;
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
