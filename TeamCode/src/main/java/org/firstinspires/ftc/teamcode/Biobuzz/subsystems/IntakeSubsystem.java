package org.firstinspires.ftc.teamcode.Biobuzz.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem for managing the Intake mechanism.
 * Features:
 * - Intake motor velocity control
 * - Depollinator servo position control (Up / Down)
 * - Two Color / Distance Sensors on the intake ramp (Ramp Back and Ramp Front)
 */
public class IntakeSubsystem {

    public enum BallColor {
        NONE,
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    // Sensor Index Constants
    public static final int SENSOR_RAMP_BACK = 0;
    public static final int SENSOR_RAMP_FRONT = 1;
    public static final int SENSOR_COUNT = 2;

    private final DcMotorEx intakeMotor;
    private final Servo depollinatorServo;

    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[SENSOR_COUNT];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[SENSOR_COUNT];

    // Default target velocity in encoder ticks per second
    public double targetVelocityTicks = IntakeConfig.DEFAULT_INTAKE_VELOCITY;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConfig.INTAKE_MOTOR_NAME);
        depollinatorServo = hardwareMap.get(Servo.class, IntakeConfig.DEPOLLINATOR_SERVO_NAME);

        // Default depollinator servo to UP position
        depollinatorUp();

        // Hardware map 2 color / distance sensors on the intake ramp
        String[] sensorNames = {
                IntakeConfig.COLOR_SENSOR_RAMP_BACK_NAME,
                IntakeConfig.COLOR_SENSOR_RAMP_FRONT_NAME
        };

        for (int i = 0; i < SENSOR_COUNT; i++) {
            try {
                colorSensors[i] = hardwareMap.get(NormalizedColorSensor.class, sensorNames[i]);
                if (colorSensors[i] instanceof DistanceSensor) {
                    distanceSensors[i] = (DistanceSensor) colorSensors[i];
                } else {
                    distanceSensors[i] = null;
                }
            } catch (Exception e) {
                colorSensors[i] = null;
                distanceSensors[i] = null;
            }
        }
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

    // --- Color & Distance Sensor Methods ---

    /**
     * Returns normalized RGBA values for a specific physical sensor index (0 = Ramp Back, 1 = Ramp Front).
     */
    public NormalizedRGBA getNormalizedColors(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT || colorSensors[sensorIndex] == null) {
            return new NormalizedRGBA();
        }
        return colorSensors[sensorIndex].getNormalizedColors();
    }

    /**
     * Returns distance in mm for a specific physical sensor index (0 = Ramp Back, 1 = Ramp Front).
     * Returns Double.MAX_VALUE if sensor is unavailable.
     */
    public double getDistanceMm(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT || distanceSensors[sensorIndex] == null) {
            return Double.MAX_VALUE;
        }
        try {
            return distanceSensors[sensorIndex].getDistance(DistanceUnit.MM);
        } catch (Exception e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Checks if a specific physical sensor index (0 = Ramp Back, 1 = Ramp Front) detects a ball.
     */
    public boolean isSensorDetectingBall(int sensorIndex) {
        double dist = getDistanceMm(sensorIndex);
        if (dist <= IntakeConfig.DISTANCE_THRESHOLD_MM) {
            return true;
        }
        NormalizedRGBA colors = getNormalizedColors(sensorIndex);
        float totalLight = colors.red + colors.green + colors.blue;
        return totalLight > 0.15f;
    }

    /**
     * Classifies the ball color detected by a specific physical sensor index (0 = Ramp Back, 1 = Ramp Front).
     */
    public BallColor detectSensorColor(int sensorIndex) {
        if (!isSensorDetectingBall(sensorIndex)) {
            return BallColor.NONE;
        }

        NormalizedRGBA colors = getNormalizedColors(sensorIndex);
        float total = colors.red + colors.green + colors.blue;
        if (total <= 0.01f) {
            return BallColor.UNKNOWN;
        }

        float redRatio = colors.red / total;
        float greenRatio = colors.green / total;
        float blueRatio = colors.blue / total;

        if (redRatio >= IntakeConfig.YELLOW_RED_MIN_RATIO && greenRatio >= IntakeConfig.YELLOW_GREEN_MIN_RATIO) {
            return BallColor.YELLOW;
        } else if (redRatio >= IntakeConfig.RED_MIN_RATIO) {
            return BallColor.RED;
        } else if (blueRatio >= IntakeConfig.BLUE_MIN_RATIO) {
            return BallColor.BLUE;
        }

        return BallColor.UNKNOWN;
    }

    // --- Ramp Back & Ramp Front Specific Convenience Methods ---

    public boolean isRampBackBallPresent() {
        return isSensorDetectingBall(SENSOR_RAMP_BACK);
    }

    public BallColor detectRampBackColor() {
        return detectSensorColor(SENSOR_RAMP_BACK);
    }

    public double getRampBackDistanceMm() {
        return getDistanceMm(SENSOR_RAMP_BACK);
    }

    public NormalizedRGBA getRampBackNormalizedColors() {
        return getNormalizedColors(SENSOR_RAMP_BACK);
    }

    public boolean isRampFrontBallPresent() {
        return isSensorDetectingBall(SENSOR_RAMP_FRONT);
    }

    public BallColor detectRampFrontColor() {
        return detectSensorColor(SENSOR_RAMP_FRONT);
    }

    public double getRampFrontDistanceMm() {
        return getDistanceMm(SENSOR_RAMP_FRONT);
    }

    public NormalizedRGBA getRampFrontNormalizedColors() {
        return getNormalizedColors(SENSOR_RAMP_FRONT);
    }

    /**
     * Returns the total count of balls detected on the intake ramp (0, 1, or 2).
     */
    public int getBallCount() {
        int count = 0;
        if (isRampBackBallPresent()) count++;
        if (isRampFrontBallPresent()) count++;
        return count;
    }

    /**
     * Returns true if any ball is detected on the intake ramp.
     */
    public boolean isRampOccupied() {
        return getBallCount() > 0;
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
