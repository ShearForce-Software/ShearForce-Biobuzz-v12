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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem for managing the Transfer mechanism and centralized Color/Distance Sensors.
 * Features:
 * - Continuous Rotation Servo to rotate the transfer mechanism
 * - REV Through Bore Encoder (read via DcMotorEx port) to track rotation, position, and velocity
 * - Centralized UpdateColorSensors() procedure querying all 7 physical color sensors (5 transfer + 2 intake ramp)
 *   and caching values in local member variables to eliminate redundant hardware I2C calls.
 * - Software PIDF velocity control (Forward Shooting Mode), predictive deceleration / indexing (Reverse Intake Mode),
 *   and closed-loop position PIDF targeting to run exact tick/slot increments.
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

    public enum TransferControlState {
        IDLE,
        MANUAL,
        VELOCITY_CONTROL,  // High-speed forward shooting PIDF
        PREDICTIVE_INTAKE,  // Predictive reverse indexing for empty slot
        POSITION_PIDF      // Closed-loop PIDF positioning to exact target ticks/slots
    }

    // Sensor Index Constants (0 to 3: Transfer, 4 to 5: Intake Ramp)
    public static final int SENSOR_TOP = 0;
    public static final int SENSOR_LEFT = 1;
    public static final int SENSOR_RIGHT = 2;
    public static final int SENSOR_BOTTOM_FRONT = 3;
    @Deprecated
    public static final int SENSOR_BOTTOM_BACK = 3;
    public static final int SENSOR_RAMP_BACK = 4;
    public static final int SENSOR_RAMP_FRONT = 5;
    public static final int SENSOR_COUNT = 6;

    private final CRServo transferServo;

    private final Servo frontGateServo;
    private final Servo backGateServo;
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

    // Software PIDF Controller State
    private TransferControlState controlState = TransferControlState.IDLE;
    private double targetVelocityTicksPerSec = 0.0;
    private double targetPositionTicks = 0.0;
    private double velIntegralSum = 0.0;
    private double lastVelError = 0.0;
    private double posIntegralSum = 0.0;
    private double lastPosError = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private final IntakeSubsystem intake;

    // Background Color Sensor Thread
    private Thread colorSensorThread = null;
    private volatile boolean colorSensorThreadRunning = false;

    public TransferSubsystem(HardwareMap hardwareMap, IntakeSubsystem intakeSubsystem) {
        intake = intakeSubsystem;

        // Continuous Rotation Servo
        transferServo = hardwareMap.get(CRServo.class, TransferConfig.SERVO_NAME);
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Position Servo for Front Gate
        frontGateServo = hardwareMap.get(Servo.class, TransferConfig.FRONT_GATE_SERVO);

        // Position Servo for Back Gate
        backGateServo = hardwareMap.get(Servo.class, TransferConfig.BACK_GATE_SERVO);

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

        // Hardware map all 6 color / distance sensors (4 transfer + 2 intake ramp)
        String[] sensorNames = {
                TransferConfig.COLOR_SENSOR_TOP_NAME,
                TransferConfig.COLOR_SENSOR_LEFT_NAME,
                TransferConfig.COLOR_SENSOR_RIGHT_NAME,
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
     * Call inside your OpMode loop to monitor sensors, PIDF calculations, and slot advancement.
     */
    public void update() {
        updateTransferPositionAndVelocity();
        updateAutoTransferLogic();
        updateRamp();
    }
    private boolean isWrongColor(BallColor color) {
        if (MatchConfig.isRedAlliance() && color == BallColor.BLUE) return true;
        if (MatchConfig.isBlueAlliance() && color == BallColor.RED) return true;
        return false;
    }

    private void updateRamp() {
        BallColor frontColor = detectRampFrontColor();
        BallColor backColor = detectRampBackColor();

        boolean wrongColorFront = isRampFrontBallPresent() && isWrongColor(frontColor);
        boolean wrongColorBack = isRampBackBallPresent() && isWrongColor(backColor);

        // 1. REJECT: Wrong color ball detected at front or back ramp -> eject ball
        if (wrongColorFront || wrongColorBack) {
            closeFrontGate();
            closeBackGate();
            if (intake != null) {
                intake.reverse();
            }
        }
        // 2. ACCEPT FRONT: Valid ball at front ramp and transfer not full
        else if (isRampFrontBallPresent() && getBallCount() < 4) {
            openFrontGate();
            closeBackGate();
        }
        // 3. ACCEPT BACK: Valid ball at back ramp and transfer not full
        else if (isRampBackBallPresent() && getBallCount() < 4) {
            openBackGate();
            closeFrontGate();
        }
        // 4. FULL: Ball at ramp but transfer is full (4 balls) -> eject extra ball
        else if ((isRampFrontBallPresent() || isRampBackBallPresent()) && isFull()) {
            closeFrontGate();
            closeBackGate();
            if (intake != null) {
                intake.reverse();
            }
        }
        // 5. DEFAULT: keep gates closed
        else {
            closeFrontGate();
            closeBackGate();
        }
    }

    private void updateAutoTransferLogic() {
        // if Auto Transfer Logic enabled (non-blocking) (toggle on/off by using SHARE + TRIANGLE)
        if (TransferConfig.AUTO_TRANSFER) {
            // if intake is turned on (bringing in new balls)
            if (intake.getVelocity() > 0) {
                // if transfer is not already in progress
                if (!IsBusy()) {
                    // if a ball is in the bottom slot
                    if (isSlotOccupied(SENSOR_BOTTOM_FRONT)) {
                        // if intake is already full then we are controlling too many balls!!!
                        if (isFull()) {
                            StowBalls();
                            // spit out any extra balls
                            intake.reverse();
                        } else {
                            Load1Ball();
                        }
                    } else if (isFull()) {
                        StowBalls();
                        // spit out anything else not being detected yet
                        intake.reverse();
                    }
                }
                // else if out-taking and velocity has reached at least 80% of target velocity and no more extra balls are being detected
            } else if ((intake.getVelocity() < -intake.targetVelocityTicks * 0.8) &&
                    !(isRampFrontBallPresent() || isRampBackBallPresent())) {
                // assume it is safe to turn off the intake subsystem
                intake.turnOff();
             }
        }
    }

    private void updateTransferPositionAndVelocity() {

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.005;

        double currentTicks = getRawTicks();
        double currentVelocity = getVelocityTicksPerSec();

        switch (controlState) {
            case VELOCITY_CONTROL: {
                double velError = targetVelocityTicksPerSec - currentVelocity;
                velIntegralSum += velError * dt;
                double velDerivative = (velError - lastVelError) / dt;
                lastVelError = velError;

                double fTerm = TransferConfig.VELOCITY_F * targetVelocityTicksPerSec;
                double pTerm = TransferConfig.VELOCITY_P * velError;
                double iTerm = TransferConfig.VELOCITY_I * velIntegralSum;
                double dTerm = TransferConfig.VELOCITY_D * velDerivative;

                double outputPower = fTerm + pTerm + iTerm + dTerm;
                setPower(Range.clip(outputPower, -1.0, 1.0));
                break;
            }

            case POSITION_PIDF: {
                double posError = targetPositionTicks - currentTicks;
                posIntegralSum += posError * dt;
                if (Math.abs(posError) < 15.0) posIntegralSum = 0.0; // Anti-windup clamping
                lastPosError = posError;

                double pTerm = TransferConfig.POSITION_P * posError;
                double iTerm = TransferConfig.POSITION_I * posIntegralSum;
                // Use direct encoder velocity for smooth derivative damping
                // This acts as an active magnetic brake that scales with how fast the transfer disc is spinning,
                // eliminating overshoot regardless of ball load
                double dTerm = -TransferConfig.POSITION_D * currentVelocity;
                // Static friction feedforward offset (kF)
                // Zero feedforward inside deadband window (<= 25 ticks) to prevent hunting/forced movement
                double fTerm = (Math.abs(posError) <= 25.0) ? 0.0 : Math.signum(posError) * TransferConfig.POSITION_F;

                double outputPower = pTerm + iTerm + dTerm + fTerm;
                outputPower = Range.clip(outputPower, -1.0, 1.0);

                // Dual settling condition: position error <= 25.0 ticks AND velocity <= 100.0 ticks/sec, or error <= 15.0 ticks
                if (Math.abs(posError) <= 15.0 || (Math.abs(posError) <= 25.0 && Math.abs(currentVelocity) <= 100.0)) {
                    stop();
                } else {
                    setPower(outputPower);
                }
                break;
            }

            case PREDICTIVE_INTAKE: {
                double remainingTicks = targetPositionTicks - currentTicks;
                double stoppingDistanceTicks = (currentVelocity * currentVelocity) / (2.0 * TransferConfig.MAX_DECEL_TICKS_PER_SEC2);

                // Dual settling condition (Position error <= 25.0 ticks AND velocity <= 50.0 ticks/sec)
                // this logic requires the target disc to be within target tolerance AND moving slowly before declaring completion
                if (Math.abs(remainingTicks) <= 25.0 && Math.abs(currentVelocity) <= 50.0) {
                    stop();
                    controlState = TransferControlState.IDLE;
                } else if (Math.abs(remainingTicks) <= Math.abs(stoppingDistanceTicks)) {
                    double posError = targetPositionTicks - currentTicks;
                    posIntegralSum += posError * dt;
                    if (Math.abs(posError) < 15.0) posIntegralSum = 0.0;
                    lastPosError = posError;

                    double pTerm = TransferConfig.POSITION_P * posError;
                    double iTerm = TransferConfig.POSITION_I * posIntegralSum;
                    double dTerm = -TransferConfig.POSITION_D * currentVelocity;
                    double fTerm = Math.signum(posError) * TransferConfig.POSITION_F;

                    double outputPower = pTerm + iTerm + dTerm + fTerm;
                    setPower(Range.clip(outputPower, -1.0, 1.0));
                } else {
                    setPower(TransferConfig.REVERSE_POWER);
                }
                break;
            }

            case IDLE:
            case MANUAL:
            default:
                if (isAdvancingSlot) {
                    isAdvance1SlotComplete();
                }
                break;
        }
    }

    /**
     * Centralized procedure to query all physical color sensors and update local cached variables.
     * This is the ONLY method permitted to make direct hardware access calls to the physical color sensors.
     */
    public void UpdateColorSensors() {
        boolean transferRotating = isRotating() || isAdvancingSlot || controlState != TransferControlState.IDLE;

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

                    // 1. Blue Check (Blue has high blueRatio)
                    if (blueRatio >= TransferConfig.BLUE_MIN_RATIO) {
                        cachedBallColor[i] = BallColor.BLUE;
                    } 
                    // 2. Yellow Check (Yellow is the ONLY ball with HIGH Green >= 0.38)
                    else if (redRatio >= TransferConfig.YELLOW_RED_MIN_RATIO && greenRatio >= TransferConfig.YELLOW_GREEN_MIN_RATIO) {
                        cachedBallColor[i] = BallColor.YELLOW;
                    } 
                    // 3. Red Check (High Red, but lower Green < 0.38)
                    else if (redRatio >= TransferConfig.RED_MIN_RATIO) {
                        cachedBallColor[i] = BallColor.RED;
                    } 
                    else {
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
        if (Math.abs(power) > 0.01 && controlState != TransferControlState.VELOCITY_CONTROL
                && controlState != TransferControlState.PREDICTIVE_INTAKE
                && controlState != TransferControlState.POSITION_PIDF) {
            controlState = TransferControlState.MANUAL;
        }
        transferServo.setPower(power);
    }

    public void rotateForward() {
        controlState = TransferControlState.MANUAL;
        setPower(TransferConfig.FORWARD_POWER);
    }

    public void rotateReverse() {
        controlState = TransferControlState.MANUAL;
        setPower(TransferConfig.REVERSE_POWER);
    }

    public void stop() {
        isAdvancingSlot = false;
        controlState = TransferControlState.IDLE;
        setPower(TransferConfig.STOP_POWER);
        // Clear cached transfer sensor detections upon stopping to prevent stale auto-transfer triggers
        for (int i = 0; i <= SENSOR_BOTTOM_FRONT; i++) {
            cachedBallPresent[i] = false;
            cachedBallColor[i] = BallColor.NONE;
        }
    }

    public boolean isRotating() {
        return Math.abs(transferServo.getPower()) > 0.01;
    }

    public double getPower() {
        return transferServo.getPower();
    }

    public TransferControlState getControlState() {
        return controlState;
    }

    // --- PIDF Velocity, Position & Predictive Control Methods ---

    /**
     * Starts high-speed forward closed-loop velocity PIDF control for shooting all balls.
     */
    public void startShootingVelocityControl() {
        controlState = TransferControlState.VELOCITY_CONTROL;
        targetVelocityTicksPerSec = TransferConfig.SHOOT_VELOCITY_TICKS_PER_SEC;
        velIntegralSum = 0.0;
        lastVelError = 0.0;
        pidTimer.reset();
    }

    /**
     * Rotates the transfer by a specified relative number of slots (1 slot = 2048 ticks) using closed-loop PIDF position control.
     * Snaps target to the nearest exact slot boundary (N * TICKS_PER_SLOT) to prevent error accumulation.
     * Positive slots rotate forward, negative slots rotate in reverse.
     *
     * @param numberOfSlots Number of slots to rotate (e.g. 1.0, 0.5, -1.0).
     */
    public void rotateSlotsPIDF(double numberOfSlots) {
        double currentTicks = getRawTicks();
        double currentSlot = currentTicks / TransferConfig.TICKS_PER_SLOT;
        double targetSlot = Math.round(currentSlot) + numberOfSlots;
        setTargetTicksPIDF(targetSlot * TransferConfig.TICKS_PER_SLOT);
    }

    /**
     * Sets an absolute target position in ticks for closed-loop PIDF position control.
     *
     * @param targetTicks Target position in ticks.
     */
    public void setTargetTicksPIDF(double targetTicks) {
        this.targetPositionTicks = targetTicks;
        this.controlState = TransferControlState.POSITION_PIDF;
        this.posIntegralSum = 0.0;
        this.lastPosError = 0.0;
        this.pidTimer.reset();
    }

    /**
     * Returns the active target position in ticks for position PIDF control.
     */
    public double getTargetPositionTicks() {
        return targetPositionTicks;
    }

    /**
     * Returns the current position error in ticks (Target - Actual).
     */
    public double getPositionError() {
        return targetPositionTicks - getRawTicks();
    }

    /**
     * Finds the nearest empty slot using color/distance sensors and uses predictive deceleration to reverse and index 1 ball.
     */
    public void intakeOneBallPredictive() {
        int emptySlotIndex = findNearestEmptySlot();
        if (emptySlotIndex == -1) {
            stop(); // All slots are full
            return;
        }

        double currentTicks = getRawTicks();
        double currentSlot = currentTicks / TransferConfig.TICKS_PER_SLOT;
        double targetSlot = Math.floor(currentSlot) - (emptySlotIndex % 4);

        targetPositionTicks = targetSlot * TransferConfig.TICKS_PER_SLOT;
        controlState = TransferControlState.PREDICTIVE_INTAKE;
        posIntegralSum = 0.0;
        lastPosError = 0.0;
        pidTimer.reset();
    }

    /**
     * Finds the first empty slot index (0 to 3) based on the stationary color/distance sensors.
     * Returns -1 if all 4 slots are occupied.
     */
    public int findNearestEmptySlot() {
        for (int i = 0; i < 4; i++) {
            if (!isSlotOccupied(i)) {
                return i;
            }
        }
        return -1;
    }

    // --- Slot Advancement Methods (1/4 Full Rotation = 1 Slot) ---

    public void advance1Slot(boolean forward) {
        this.advanceForward = forward;
        this.isAdvancingSlot = true;
        rotateSlotsPIDF(forward ? 1.0 : -1.0);
    }

    /**
     * Rotates the transfer forward by 1 slot (1/4 full rotation) using PIDF position control.
     */
    public void Shoot1Ball() {
        advance1Slot(true);
    }

    /**
     * Rotates the transfer forward by 5 slots (1 full rotation + 1/4 rotation) using PIDF position control.
     */
    public void ShootAllBalls() {
        this.advanceForward = true;
        this.isAdvancingSlot = true;
        rotateSlotsPIDF(5.0);
    }

    /**
     * Rotates the transfer in reverse by 1 slot (1/4 full rotation) using PIDF position control.
     */
    public void Load1Ball() {
        advance1Slot(false);
    }

    /**
     * Rotates the transfer in reverse by 1/2 slot (1/8 full rotation) using PIDF position control
     * to keep bottom ball from falling out until ready to shoot.
     */
    public void StowBalls() {
        this.advanceForward = false;
        this.isAdvancingSlot = true;
        rotateSlotsPIDF(-0.5);
    }

    public boolean IsBusy() {
        return controlState != TransferControlState.IDLE || isAdvancingSlot;
    }

    /**
     * Returns the transfer to the zero position (ready to take in first ball) using PIDF position control.
     */
    public void ReturnToZeroPosition() {
        this.advanceForward = false;
        this.isAdvancingSlot = true;
        setTargetTicksPIDF(0.0);
    }

    public boolean isAdvance1SlotComplete() {
        if (controlState == TransferControlState.IDLE) {
            isAdvancingSlot = false;
            return true;
        }
        return false;
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
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffsetTicks = 0;
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
            case 3: return isSensorDetectingBall(SENSOR_BOTTOM_FRONT);
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
            case 3: return detectSensorColor(SENSOR_BOTTOM_FRONT);
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


    public Command stopCommand() {
        return instant(this::stop);
    }



    /**
     * Ivy Command to shoot all balls by rotating 5 slots forward using closed-loop PIDF position control,
     * with a configurable safety timeout.
     *
     * @param timeoutMs Maximum duration in milliseconds to wait before timing out.
     */
    public Command shootAllBallsCommand(long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        return sequential(
                instant(() -> {
                    timer.reset();
                    ShootAllBalls();
                }),
                waitUntil(() -> !IsBusy() || timer.milliseconds() >= timeoutMs)
        );
    }

    /**
     * Ivy Command to shoot all balls with a default 3000ms (3.0 second) safety timeout.
     */
    public Command shootAllBallsCommand() {
        return shootAllBallsCommand(3000);
    }

    // --- Background Color Sensor Thread Control ---

    /**
     * Starts a dedicated asynchronous background thread for non-blocking I2C color sensor updates (~15 Hz).
     */
    public void startColorSensorThread() {
        if (colorSensorThreadRunning) return;
        colorSensorThreadRunning = true;
        colorSensorThread = new Thread(() -> {
            while (colorSensorThreadRunning && !Thread.currentThread().isInterrupted()) {
                try {
                    UpdateColorSensors();
                    Thread.sleep(60);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // Ignore transient exceptions
                }
            }
        }, "TransferColorSensorBackgroundThread");
        colorSensorThread.start();
    }

    /**
     * Safely stops the background color sensor thread.
     */
    public void stopColorSensorThread() {
        colorSensorThreadRunning = false;
        if (colorSensorThread != null) {
            colorSensorThread.interrupt();
            colorSensorThread = null;
        }
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

    // --- Back Gate Servo Control Methods ---
    public void openBackGate() {
        backGateServo.setPosition(TransferConfig.BACK_GATE_OPEN_POS);
    }

    public void closeBackGate() {
        backGateServo.setPosition(TransferConfig.BACK_GATE_CLOSE_POS);
    }

    public void setBackGatePosition(double position) {
        backGateServo.setPosition(position);
    }

    public double getBackGatePosition() {
        return backGateServo.getPosition();
    }

}
