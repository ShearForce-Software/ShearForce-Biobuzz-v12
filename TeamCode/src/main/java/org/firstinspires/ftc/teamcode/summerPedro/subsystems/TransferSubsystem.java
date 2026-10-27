package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem for managing the Transfer mechanism.
 * Features:
 * - Continuous Rotation Servo to rotate the transfer mechanism
 * - REV Through Bore Encoder (read via DcMotorEx port) to track rotation, position, and velocity
 * - Four Color / Distance Sensors to detect ball placement, color, and occupancy across transfer slots
 */
public class TransferSubsystem {

    public enum BallColor {
        NONE,
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private final CRServo transferServo;
    private final DcMotorEx encoderMotor;

    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[4];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[4];

    private double encoderOffsetTicks = 0.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        // Continuous Rotation Servo
        transferServo = hardwareMap.get(CRServo.class, TransferConfig.SERVO_NAME);

        // REV Through Bore Encoder plugged into a Control/Expansion Hub motor encoder port
        encoderMotor = hardwareMap.get(DcMotorEx.class, TransferConfig.ENCODER_NAME);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Hardware map four color / distance sensors
        String[] sensorNames = {
                TransferConfig.COLOR_SENSOR_1_NAME,
                TransferConfig.COLOR_SENSOR_2_NAME,
                TransferConfig.COLOR_SENSOR_3_NAME,
                TransferConfig.COLOR_SENSOR_4_NAME
        };

        for (int i = 0; i < 4; i++) {
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

    // --- Servo Control Methods ---

    /**
     * Sets raw power to the continuous rotation transfer servo (-1.0 to 1.0).
     */
    public void setPower(double power) {
        transferServo.setPower(power);
    }

    /**
     * Rotates the transfer forward using configured power.
     */
    public void rotateForward() {
        setPower(TransferConfig.FORWARD_POWER);
    }

    /**
     * Rotates the transfer in reverse using configured power.
     */
    public void rotateReverse() {
        setPower(TransferConfig.REVERSE_POWER);
    }

    /**
     * Stops the transfer continuous rotation servo.
     */
    public void stop() {
        setPower(TransferConfig.STOP_POWER);
    }

    /**
     * Returns true if the transfer servo is active.
     */
    public boolean isRotating() {
        return Math.abs(transferServo.getPower()) > 0.01;
    }

    /**
     * Returns current servo power level.
     */
    public double getPower() {
        return transferServo.getPower();
    }

    // --- REV Through Bore Encoder Methods ---

    /**
     * Returns the raw encoder ticks, taking inversion and zero offset into account.
     */
    public double getRawTicks() {
        double raw = encoderMotor.getCurrentPosition();
        if (TransferConfig.ENCODER_REVERSED) {
            raw = -raw;
        }
        return raw - encoderOffsetTicks;
    }

    /**
     * Resets the encoder position count to zero.
     */
    public void resetEncoder() {
        double raw = encoderMotor.getCurrentPosition();
        if (TransferConfig.ENCODER_REVERSED) {
            raw = -raw;
        }
        encoderOffsetTicks = raw;
    }

    /**
     * Returns the total rotations tracked by the REV Through Bore Encoder.
     */
    public double getRotations() {
        return getRawTicks() / TransferConfig.TICKS_PER_REV;
    }

    /**
     * Returns total position in degrees.
     */
    public double getDegrees() {
        return getRotations() * 360.0;
    }

    /**
     * Returns total position in radians.
     */
    public double getRadians() {
        return getRotations() * 2.0 * Math.PI;
    }

    /**
     * Returns encoder velocity in ticks per second.
     */
    public double getVelocityTicksPerSec() {
        double vel = encoderMotor.getVelocity();
        return TransferConfig.ENCODER_REVERSED ? -vel : vel;
    }

    /**
     * Returns encoder rotation speed in RPM.
     */
    public double getVelocityRPM() {
        return (getVelocityTicksPerSec() / TransferConfig.TICKS_PER_REV) * 60.0;
    }

    // --- Color & Distance Sensor Methods ---

    /**
     * Returns normalized RGBA values for a specific color sensor slot (0 to 3).
     */
    public NormalizedRGBA getNormalizedColors(int slotIndex) {
        if (slotIndex < 0 || slotIndex >= 4 || colorSensors[slotIndex] == null) {
            return new NormalizedRGBA();
        }
        return colorSensors[slotIndex].getNormalizedColors();
    }

    /**
     * Returns distance in mm for a specific color sensor slot (0 to 3).
     * Returns Double.MAX_VALUE if sensor is unavailable.
     */
    public double getDistanceMm(int slotIndex) {
        if (slotIndex < 0 || slotIndex >= 4 || distanceSensors[slotIndex] == null) {
            return Double.MAX_VALUE;
        }
        try {
            return distanceSensors[slotIndex].getDistance(DistanceUnit.MM);
        } catch (Exception e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Checks if a ball is present at a specific sensor slot (0 to 3).
     */
    public boolean isBallPresent(int slotIndex) {
        double dist = getDistanceMm(slotIndex);
        if (dist <= TransferConfig.DISTANCE_THRESHOLD_MM) {
            return true;
        }
        // Fallback check using total alpha/luminosity if distance sensor isn't available
        NormalizedRGBA colors = getNormalizedColors(slotIndex);
        float totalLight = colors.red + colors.green + colors.blue;
        return totalLight > 0.15f;
    }

    /**
     * Classifies the color of the ball present at a specific sensor slot (0 to 3).
     */
    public BallColor detectBallColor(int slotIndex) {
        if (!isBallPresent(slotIndex)) {
            return BallColor.NONE;
        }

        NormalizedRGBA colors = getNormalizedColors(slotIndex);
        float total = colors.red + colors.green + colors.blue;
        if (total <= 0.01f) {
            return BallColor.UNKNOWN;
        }

        float redRatio = colors.red / total;
        float greenRatio = colors.green / total;
        float blueRatio = colors.blue / total;

        if (redRatio >= TransferConfig.YELLOW_RED_MIN_RATIO && greenRatio >= TransferConfig.YELLOW_GREEN_MIN_RATIO) {
            return BallColor.YELLOW;
        } else if (redRatio >= TransferConfig.RED_MIN_RATIO) {
            return BallColor.RED;
        } else if (blueRatio >= TransferConfig.BLUE_MIN_RATIO) {
            return BallColor.BLUE;
        }

        return BallColor.UNKNOWN;
    }

    /**
     * Returns an array of size 4 indicating ball presence for all slots.
     */
    public boolean[] getSlotOccupancy() {
        boolean[] occupancy = new boolean[4];
        for (int i = 0; i < 4; i++) {
            occupancy[i] = isBallPresent(i);
        }
        return occupancy;
    }

    /**
     * Returns an array of size 4 containing the detected BallColor for each slot.
     */
    public BallColor[] getSlotColors() {
        BallColor[] colors = new BallColor[4];
        for (int i = 0; i < 4; i++) {
            colors[i] = detectBallColor(i);
        }
        return colors;
    }

    /**
     * Returns the total count of balls currently detected in the transfer (0 to 4).
     */
    public int getBallCount() {
        int count = 0;
        for (int i = 0; i < 4; i++) {
            if (isBallPresent(i)) {
                count++;
            }
        }
        return count;
    }

    /**
     * Returns true if all 4 transfer slots are occupied.
     */
    public boolean isFull() {
        return getBallCount() >= 4;
    }

    /**
     * Returns true if no transfer slots are occupied.
     */
    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    // --- Ivy Command Generators ---

    /**
     * Command to rotate the transfer forward.
     */
    public Command rotateForwardCommand() {
        return instant(this::rotateForward);
    }

    /**
     * Command to rotate the transfer in reverse.
     */
    public Command rotateReverseCommand() {
        return instant(this::rotateReverse);
    }

    /**
     * Command to stop the transfer servo.
     */
    public Command stopCommand() {
        return instant(this::stop);
    }

    /**
     * Sequence command: rotates transfer forward until all 4 ball slots are filled, then stops.
     */
    public Command transferUntilFullCommand() {
        return sequential(
                instant(this::rotateForward),
                waitUntil(this::isFull),
                instant(this::stop)
        );
    }
}
