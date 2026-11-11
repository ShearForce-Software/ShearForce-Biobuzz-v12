package org.firstinspires.ftc.teamcode.Biobuzz.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * - 4 Ball Slots (Top, Left, Right, Bottom) with 5 Color / Distance Sensors:
 *   - Top Slot (Sensor 0)
 *   - Left Slot (Sensor 1)
 *   - Right Slot (Sensor 2)
 *   - Bottom Slot (Sensor 3 = Bottom Back, Sensor 4 = Bottom Front)
 */
public class TransferSubsystem {

    public enum BallColor {
        NONE,
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public enum Slot {
        TOP(0, "Top"),
        LEFT(1, "Left"),
        RIGHT(2, "Right"),
        BOTTOM(3, "Bottom");

        public final int index;
        public final String name;

        Slot(int index, String name) {
            this.index = index;
            this.name = name;
        }
    }

    // Sensor Index Constants
    public static final int SENSOR_TOP = 0;
    public static final int SENSOR_LEFT = 1;
    public static final int SENSOR_RIGHT = 2;
    public static final int SENSOR_BOTTOM_BACK = 3;
    public static final int SENSOR_BOTTOM_FRONT = 4;
    public static final int SENSOR_COUNT = 5;

    private final CRServo transferServo;
    private final DcMotorEx encoderMotor;

    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[SENSOR_COUNT];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[SENSOR_COUNT];

    private double encoderOffsetTicks = 0.0;

    // Advance 1 slot tracking
    private boolean isAdvancingSlot = false;
    private boolean advanceForward = true;
    private double targetEncoderTicks = 0.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        // Continuous Rotation Servo
        transferServo = hardwareMap.get(CRServo.class, TransferConfig.SERVO_NAME);

        // REV Through Bore Encoder plugged into a Control/Expansion Hub motor encoder port
        encoderMotor = hardwareMap.get(DcMotorEx.class, TransferConfig.ENCODER_NAME);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hardware map 5 color / distance sensors
        String[] sensorNames = {
                TransferConfig.COLOR_SENSOR_TOP_NAME,
                TransferConfig.COLOR_SENSOR_LEFT_NAME,
                TransferConfig.COLOR_SENSOR_RIGHT_NAME,
                TransferConfig.COLOR_SENSOR_BOTTOM_BACK_NAME,
                TransferConfig.COLOR_SENSOR_BOTTOM_FRONT_NAME
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
        isAdvancingSlot = false;
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

    // --- Slot Advancement Methods (1/4 Full Rotation = 1 Slot) ---

    /**
     * Rotates the transfer 1/4 of a full rotation (1 slot) forward or reverse based on TransferConfig.TICKS_PER_REV.
     *
     * @param forward true to rotate forward 1 slot, false to rotate reverse 1 slot.
     */
    public void advance1Slot(boolean forward) {
        this.advanceForward = forward;
        double quarterRotationTicks = TransferConfig.TICKS_PER_REV / 4.0;
        if (forward) {
            targetEncoderTicks = getRawTicks() + quarterRotationTicks;
            rotateForward();
        } else {
            targetEncoderTicks = getRawTicks() - quarterRotationTicks;
            rotateReverse();
        }
        isAdvancingSlot = true;
    }

    /**
     * Rotates the transfer forward by 1 slot (1/4 full rotation).
     */
    public void Shoot1Ball() {
        advance1Slot(true);
    }

    /**
     * Rotates the transfer forward by 5 slots (1 full rotation + 1/4 rotation for extra measure).
     */
    public void ShootAllBalls() {
        this.advanceForward = true;
        double quarterRotationTicks = TransferConfig.TICKS_PER_REV / 4.0;

        // rotate 5 slots (all balls should shoot out)
        targetEncoderTicks = getRawTicks() + TransferConfig.TICKS_PER_REV + quarterRotationTicks;
        rotateForward();

        isAdvancingSlot = true;

    }

    /**
     * Rotates the transfer in reverse by 1 slot (1/4 full rotation).
     */
    public void Load1Ball() {
        advance1Slot(false);
    }

    /**
     * Rotates the transfer in reverse by 1/2 slot (1/8 full rotation)
     * to keep bottom ball from falling out until ready to shoot
     */
    public void StowBalls() {
        this.advanceForward = false;
        double eighthRotationTicks = TransferConfig.TICKS_PER_REV / 8.0;
        targetEncoderTicks = getRawTicks() - eighthRotationTicks;
        rotateReverse();

        isAdvancingSlot = true;
    }

    public boolean IsBusy() {
        return isAdvancingSlot;
    }

    /**
     * Returns the transfer to the zero position (ready to take in first ball)
     */
    public void ReturnToZeroPosition() {
        targetEncoderTicks = 0;

        if (targetEncoderTicks != getRawTicks()) {
            if (targetEncoderTicks > getRawTicks()) {
                this.advanceForward = true;
                rotateForward();
            } else {
                this.advanceForward = false;
                rotateReverse();
            }
            isAdvancingSlot = true;
        }
    }

    /**
     * Checks if the 1 slot (1/4 rotation) movement has completed and stops the servo when target is reached.
     *
     * @return true if movement is finished or inactive.
     */
    public boolean isAdvance1SlotComplete() {
        if (!isAdvancingSlot) {
            return true;
        }

        // if we were shooting
        if (advanceForward) {
            if (getRawTicks() >= targetEncoderTicks) {
                stop();

                if (TransferConfig.AUTO_TRANSFER) {
                    ReturnToZeroPosition();
                }
                else {
                    return true;
                }
            }
        } else {
            if (getRawTicks() <= targetEncoderTicks) {
                stop();
                return true;
            }
        }
        return false;
    }

    /**
     * Call inside your OpMode loop to monitor and complete active 1-slot advancement movements.
     */
    public void update() {
        if (isAdvancingSlot) {
            isAdvance1SlotComplete();
        }
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

    // --- Individual Physical Sensor Methods (0 to 4) ---

    /**
     * Returns normalized RGBA values for a specific physical sensor index (0 to 4).
     */
    public NormalizedRGBA getNormalizedColors(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT || colorSensors[sensorIndex] == null) {
            return new NormalizedRGBA();
        }
        return colorSensors[sensorIndex].getNormalizedColors();
    }

    /**
     * Returns distance in mm for a specific physical sensor index (0 to 4).
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
     * Checks if a specific physical sensor index (0 to 4) detects a ball.
     */
    public boolean isSensorDetectingBall(int sensorIndex) {
        double dist = getDistanceMm(sensorIndex);
        if (dist <= TransferConfig.DISTANCE_THRESHOLD_MM) {
            return true;
        }
        NormalizedRGBA colors = getNormalizedColors(sensorIndex);
        float totalLight = colors.red + colors.green + colors.blue;
        return totalLight > 0.15f;
    }

    /**
     * Classifies the ball color detected by a specific physical sensor index (0 to 4).
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

        if (redRatio >= TransferConfig.YELLOW_RED_MIN_RATIO && greenRatio >= TransferConfig.YELLOW_GREEN_MIN_RATIO) {
            return BallColor.YELLOW;
        } else if (redRatio >= TransferConfig.RED_MIN_RATIO) {
            return BallColor.RED;
        } else if (blueRatio >= TransferConfig.BLUE_MIN_RATIO) {
            return BallColor.BLUE;
        }

        return BallColor.UNKNOWN;
    }

    // --- Slot-Based Methods (Top=0, Left=1, Right=2, Bottom=3) ---

    /**
     * Checks if a ball is present in a specific slot (0=Top, 1=Left, 2=Right, 3=Bottom).
     * Note: Bottom slot (index 3) is occupied if EITHER Bottom Back OR Bottom Front sensor detects a ball.
     */
    public boolean isSlotOccupied(int slotIndex) {
        switch (slotIndex) {
            case 0: return isSensorDetectingBall(SENSOR_TOP);
            case 1: return isSensorDetectingBall(SENSOR_LEFT);
            case 2: return isSensorDetectingBall(SENSOR_RIGHT);
            case 3: return isSensorDetectingBall(SENSOR_BOTTOM_BACK) || isSensorDetectingBall(SENSOR_BOTTOM_FRONT);
            default: return false;
        }
    }

    public boolean isSlotOccupied(Slot slot) {
        return isSlotOccupied(slot.index);
    }

    /**
     * Alias method for backward compatibility.
     */
    public boolean isBallPresent(int slotIndex) {
        return isSlotOccupied(slotIndex);
    }

    /**
     * Classifies the color of the ball present in a specific slot (0=Top, 1=Left, 2=Right, 3=Bottom).
     */
    public BallColor detectSlotColor(int slotIndex) {
        switch (slotIndex) {
            case 0: return detectSensorColor(SENSOR_TOP);
            case 1: return detectSensorColor(SENSOR_LEFT);
            case 2: return detectSensorColor(SENSOR_RIGHT);
            case 3:
                if (isSensorDetectingBall(SENSOR_BOTTOM_BACK)) {
                    return detectSensorColor(SENSOR_BOTTOM_BACK);
                } else if (isSensorDetectingBall(SENSOR_BOTTOM_FRONT)) {
                    return detectSensorColor(SENSOR_BOTTOM_FRONT);
                } else {
                    return detectSensorColor(SENSOR_BOTTOM_BACK);
                }
            default: return BallColor.NONE;
        }
    }

    public BallColor detectSlotColor(Slot slot) {
        return detectSlotColor(slot.index);
    }

    /**
     * Alias method for backward compatibility.
     */
    public BallColor detectBallColor(int slotIndex) {
        return detectSlotColor(slotIndex);
    }

    /**
     * Returns an array of size 4 indicating slot occupancy (Top, Left, Right, Bottom).
     */
    public boolean[] getSlotOccupancy() {
        boolean[] occupancy = new boolean[4];
        for (int i = 0; i < 4; i++) {
            occupancy[i] = isSlotOccupied(i);
        }
        return occupancy;
    }

    /**
     * Returns an array of size 4 containing the detected BallColor for each slot (Top, Left, Right, Bottom).
     */
    public BallColor[] getSlotColors() {
        BallColor[] colors = new BallColor[4];
        for (int i = 0; i < 4; i++) {
            colors[i] = detectSlotColor(i);
        }
        return colors;
    }

    /**
     * Returns the total count of occupied slots in the transfer (0 to 4).
     */
    public int getBallCount() {
        int count = 0;
        for (int i = 0; i < 4; i++) {
            if (isSlotOccupied(i)) {
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
     * Command to advance the transfer by 1 slot (1/4 full rotation).
     *
     * @param forward true for forward 1 slot, false for reverse 1 slot.
     */
    public Command advance1SlotCommand(boolean forward) {
        return sequential(
                instant(() -> advance1Slot(forward)),
                waitUntil(this::isAdvance1SlotComplete)
        );
    }

    /**
     * Command to advance the transfer forward by 1 slot (1/4 full rotation).
     */
    public Command advance1SlotForwardCommand() {
        return advance1SlotCommand(true);
    }

    /**
     * Command to advance the transfer in reverse by 1 slot (1/4 full rotation).
     */
    public Command advance1SlotReverseCommand() {
        return advance1SlotCommand(false);
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
