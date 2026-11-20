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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem for managing the Transfer mechanism and centralized Color/Distance Sensors.
 * Features:
 * - Continuous Rotation Servo to rotate the transfer mechanism
 * - REV Through Bore Encoder (read via DcMotorEx port) to track rotation, position, and velocity
 * - Centralized UpdateColorSensors() procedure querying all 7 physical color sensors (5 transfer + 2 intake ramp)
 *   and caching values in local member variables to eliminate redundant hardware I2C calls.
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

    // Sensor Index Constants (0 to 4: Transfer, 5 to 6: Intake Ramp)
    public static final int SENSOR_TOP = 0;
    public static final int SENSOR_LEFT = 1;
    public static final int SENSOR_RIGHT = 2;
    public static final int SENSOR_BOTTOM_BACK = 3;
    public static final int SENSOR_BOTTOM_FRONT = 4;
    public static final int SENSOR_RAMP_BACK = 5;
    public static final int SENSOR_RAMP_FRONT = 6;
    public static final int SENSOR_COUNT = 7;

    private final CRServo transferServo;

    private final Servo frontGateServo;
    private final DcMotorEx encoderMotor;

    // Physical hardware sensors (queried ONLY in UpdateColorSensors)
    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[SENSOR_COUNT];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[SENSOR_COUNT];

    // Local cached variables storing color sensor data
    private final double[] cachedDistanceMm = new double[SENSOR_COUNT];
    private final NormalizedRGBA[] cachedNormalizedColors = new NormalizedRGBA[SENSOR_COUNT];
    private final boolean[] cachedBallPresent = new boolean[SENSOR_COUNT];
    private final BallColor[] cachedBallColor = new BallColor[SENSOR_COUNT];

    // Circuit Breaker for I2C disconnection resilience (prevents I2C bus starvation)
    private final int[] sensorFailureCounts = new int[SENSOR_COUNT];
    private final ElapsedTime[] sensorCooldownTimers = new ElapsedTime[SENSOR_COUNT];
    private static final int MAX_CONSECUTIVE_FAILURES = 5;
    private static final double SENSOR_COOLDOWN_SECONDS = 5.0;

    private double encoderOffsetTicks = 0.0;

    // Advance 1 slot tracking
    private boolean isAdvancingSlot = false;
    private boolean advanceForward = true;
    private double targetEncoderTicks = 0.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        // Continuous Rotation Servo
        transferServo = hardwareMap.get(CRServo.class, TransferConfig.SERVO_NAME);

        // Position Servo for Front Gate
        frontGateServo = hardwareMap.get(Servo.class, TransferConfig.FRONT_GATE_SERVO);

        // REV Through Bore Encoder plugged into a Control/Expansion Hub motor encoder port
        encoderMotor = hardwareMap.get(DcMotorEx.class, TransferConfig.ENCODER_NAME);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        encoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize cached normalized colors array and circuit breaker timers
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            cachedNormalizedColors[i] = new NormalizedRGBA();
            cachedDistanceMm[i] = Double.MAX_VALUE;
            cachedBallColor[i] = BallColor.NONE;
            cachedBallPresent[i] = false;
            sensorCooldownTimers[i] = new ElapsedTime();
            sensorFailureCounts[i] = 0;
        }

        // Hardware map all 7 color / distance sensors (5 transfer + 2 intake ramp)
        String[] sensorNames = {
                TransferConfig.COLOR_SENSOR_TOP_NAME,
                TransferConfig.COLOR_SENSOR_LEFT_NAME,
                TransferConfig.COLOR_SENSOR_RIGHT_NAME,
                TransferConfig.COLOR_SENSOR_BOTTOM_BACK_NAME,
                TransferConfig.COLOR_SENSOR_BOTTOM_FRONT_NAME,
                TransferConfig.COLOR_SENSOR_RAMP_BACK_NAME,
                TransferConfig.COLOR_SENSOR_RAMP_FRONT_NAME
        };

        for (int i = 0; i < SENSOR_COUNT; ++i) {
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

        // Perform initial sensor read to populate cache
        UpdateColorSensors();
    }

    /**
     * Centralized procedure to query all physical color sensors and update local cached variables.
     * This is the ONLY method permitted to make direct hardware access calls to the physical color sensors.
     */
    public void UpdateColorSensors() {
        boolean transferRotating = isRotating() || isAdvancingSlot;

        for (int i = 0; i < SENSOR_COUNT; ++i) {
            // Check if this is a transfer ring sensor (slots 0 to 4: Top, Left, Right, Bot Back, Bot Front)
            boolean isTransferSensor = (i <= SENSOR_BOTTOM_FRONT);

            // Skip transfer ring sensors if transfer is rotating to avoid false detections from arms passing sensors.
            // Ramp sensors (indices 5 & 6) are ALWAYS checked regardless of transfer rotation.
            if (isTransferSensor && transferRotating) {
                //cachedDistanceMm[i] = Double.MAX_VALUE;
                //cachedNormalizedColors[i] = new NormalizedRGBA();
                //cachedBallPresent[i] = false;
                //cachedBallColor[i] = BallColor.NONE;
                // Skip doing anything with these sensors, just leave the cached data alone
                continue;
            }

            // Circuit Breaker: Skip query if sensor has repeatedly failed and cooldown timer is active
            if (sensorFailureCounts[i] >= MAX_CONSECUTIVE_FAILURES) {
                if (sensorCooldownTimers[i].seconds() < SENSOR_COOLDOWN_SECONDS) {
                    cachedDistanceMm[i] = Double.MAX_VALUE;
                    cachedNormalizedColors[i] = new NormalizedRGBA();
                    cachedBallPresent[i] = false;
                    cachedBallColor[i] = BallColor.NONE;
                    continue;
                } else {
                    // Cooldown expired; attempt to retry reading sensor
                    sensorFailureCounts[i] = 0;
                }
            }

            boolean readFailed = false;

            // 1. Read distance first
            if (distanceSensors[i] != null) {
                try {
                    cachedDistanceMm[i] = distanceSensors[i].getDistance(DistanceUnit.MM);
                } catch (Exception e) {
                    cachedDistanceMm[i] = Double.MAX_VALUE;
                    readFailed = true;
                }
            } else {
                cachedDistanceMm[i] = Double.MAX_VALUE;
            }

            // Check if ball is present based on distance reading
            boolean present = (cachedDistanceMm[i] <= TransferConfig.DISTANCE_THRESHOLD_MM);
            cachedBallPresent[i] = present;

            if (present) {
                // 2. Only read normalized colors if a ball is detected by the distance sensor to save I2C time
                if (colorSensors[i] != null) {
                    try {
                        cachedNormalizedColors[i] = colorSensors[i].getNormalizedColors();
                    } catch (Exception e) {
                        cachedNormalizedColors[i] = new NormalizedRGBA();
                        readFailed = true;
                    }
                } else {
                    cachedNormalizedColors[i] = new NormalizedRGBA();
                }

                NormalizedRGBA rgba = cachedNormalizedColors[i];
                float totalLight = rgba.red + rgba.green + rgba.blue;

                if (totalLight <= 0.01f) {
                    cachedBallColor[i] = BallColor.NONE;
                } else {
                    float redRatio = rgba.red / totalLight;
                    float greenRatio = rgba.green / totalLight;
                    float blueRatio = rgba.blue / totalLight;

                    if (redRatio >= TransferConfig.YELLOW_RED_MIN_RATIO && greenRatio >= TransferConfig.YELLOW_GREEN_MIN_RATIO) {
                        cachedBallColor[i] = BallColor.YELLOW;
                    } else if (redRatio >= TransferConfig.RED_MIN_RATIO) {
                        cachedBallColor[i] = BallColor.RED;
                    } else if (blueRatio >= TransferConfig.BLUE_MIN_RATIO) {
                        cachedBallColor[i] = BallColor.BLUE;
                    } else {
                        cachedBallColor[i] = BallColor.UNKNOWN;
                    }
                }
            } else {
                // Ball is not present: skip reading color sensor and set color to NONE
                cachedNormalizedColors[i] = new NormalizedRGBA();
                cachedBallColor[i] = BallColor.NONE;
            }

            if (readFailed) {
                sensorFailureCounts[i]++;
                if (sensorFailureCounts[i] >= MAX_CONSECUTIVE_FAILURES) {
                    sensorCooldownTimers[i].reset();
                }
            } else {
                sensorFailureCounts[i] = 0;
            }
        }
    }

    // --- Servo Control Methods ---

    public void setPower(double power) {
        transferServo.setPower(power);
    }

    public void rotateForward() {
        setPower(TransferConfig.FORWARD_POWER);
    }

    public void rotateReverse() {
        setPower(TransferConfig.REVERSE_POWER);
    }

    public void stop() {
        isAdvancingSlot = false;
        setPower(TransferConfig.STOP_POWER);
    }

    public boolean isRotating() {
        return Math.abs(transferServo.getPower()) > 0.01;
    }

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
                } else {
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

    public double getRawTicks() {
        double raw = encoderMotor.getCurrentPosition();
        if (TransferConfig.ENCODER_REVERSED) {
            raw = -raw;
        }
        return raw - encoderOffsetTicks;
    }

    public void resetEncoder() {
        double raw = encoderMotor.getCurrentPosition();
        if (TransferConfig.ENCODER_REVERSED) {
            raw = -raw;
        }
        encoderOffsetTicks = raw;
    }

    public double getRotations() {
        return getRawTicks() / TransferConfig.TICKS_PER_REV;
    }

    public double getDegrees() {
        return getRotations() * 360.0;
    }

    public double getRadians() {
        return getRotations() * 2.0 * Math.PI;
    }

    public double getVelocityTicksPerSec() {
        double vel = encoderMotor.getVelocity();
        return TransferConfig.ENCODER_REVERSED ? -vel : vel;
    }

    public double getVelocityRPM() {
        return (getVelocityTicksPerSec() / TransferConfig.TICKS_PER_REV) * 60.0;
    }

    // --- Accessor Methods using Stored Local Variables (NO direct hardware calls) ---

    public NormalizedRGBA getNormalizedColors(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT) {
            return new NormalizedRGBA();
        }
        return cachedNormalizedColors[sensorIndex];
    }

    /**
     * Returns distance in mm for a specific physical sensor index 
     * Returns Double.MAX_VALUE if sensor is unavailable.
     */
    public double getDistanceMm(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT) {
            return Double.MAX_VALUE;
        }
        return cachedDistanceMm[sensorIndex];
    }

    public boolean isSensorDetectingBall(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT) {
            return false;
        }
        return cachedBallPresent[sensorIndex];
    }

    public BallColor detectSensorColor(int sensorIndex) {
        if (sensorIndex < 0 || sensorIndex >= SENSOR_COUNT) {
            return BallColor.NONE;
        }
        return cachedBallColor[sensorIndex];
    }

    // --- Slot-Based Methods (Top=0, Left=1, Right=2, Bottom=3) ---

    public boolean isSlotOccupied(int slotIndex) {
        switch (slotIndex) {
            case 0: return isSensorDetectingBall(SENSOR_TOP);
            case 1: return isSensorDetectingBall(SENSOR_LEFT);
            case 2: return isSensorDetectingBall(SENSOR_RIGHT);
            case 3: return isSensorDetectingBall(SENSOR_BOTTOM_BACK);
            case 4: return isSensorDetectingBall(SENSOR_BOTTOM_FRONT);
            default: return false;
        }
    }

    public boolean isSlotOccupied(Slot slot) {
        return isSlotOccupied(slot.index);
    }

    public boolean isBallPresent(int slotIndex) {
        return isSlotOccupied(slotIndex);
    }

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

    public BallColor detectBallColor(int slotIndex) {
        return detectSlotColor(slotIndex);
    }

    public boolean[] getSlotOccupancy() {
        boolean[] occupancy = new boolean[4];
        for (int i = 0; i < 4; ++i) {
            occupancy[i] = isSlotOccupied(i);
        }
        return occupancy;
    }

    public BallColor[] getSlotColors() {
        BallColor[] colors = new BallColor[4];
        for (int i = 0; i < 4; ++i) {
            colors[i] = detectSlotColor(i);
        }
        return colors;
    }

    public int getBallCount() {
        int count = 0;
        for (int i = 0; i < 4; ++i) {
            if (isSlotOccupied(i)) {
                ++count;
            }
        }
        return count;
    }

    public boolean isFull() {
        return getBallCount() >= 4;
    }

    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    // --- Intake Ramp Sensor Convenience Methods (Delegated to Stored Variables) ---

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

    public int getRampBallCount() {
        int count = 0;
        if (isRampBackBallPresent()) ++count;
        if (isRampFrontBallPresent()) ++count;
        return count;
    }

    public boolean isRampOccupied() {
        return getRampBallCount() > 0;
    }

    // --- Ivy Command Generators ---

    public Command rotateForwardCommand() {
        return instant(this::rotateForward);
    }

    public Command rotateReverseCommand() {
        return instant(this::rotateReverse);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public Command advance1SlotCommand(boolean forward) {
        return sequential(
                instant(() -> advance1Slot(forward)),
                waitUntil(this::isAdvance1SlotComplete)
        );
    }

    public Command advance1SlotForwardCommand() {
        return advance1SlotCommand(true);
    }

    public Command advance1SlotReverseCommand() {
        return advance1SlotCommand(false);
    }

    public Command transferUntilFullCommand() {
        return sequential(
                instant(this::rotateForward),
                waitUntil(this::isFull),
                instant(this::stop)
        );
    }
    // --- Front Gate Servo Control Methods ---
    public void openFrontGate() {
        frontGateServo.setPosition(TransferConfig.FRONT_GATE_OPEN_POS);
    }

    public void closeFrontGate() {
        frontGateServo.setPosition(TransferConfig.FRONT_GATE_CLOSED_POS);
    }

    public void setFrontGatePosition(double position) {
        frontGateServo.setPosition(position);
    }

    public double getFrontGatePosition() {
        return frontGateServo.getPosition();
    }

    public Command openFrontGateCommand() {
        return instant(this::openFrontGate);
    }

    public Command closeFrontGateCommand() {
        return instant(this::closeFrontGate);
    }

    public void updateRamp(IntakeSubsystem intake) {
        if (getBallCount() < 3) {
            openFrontGate();
        } else {
            closeFrontGate();
            if (intake != null && isRampFrontBallPresent()) {
                intake.reverse();
            }
        }
    }
}
