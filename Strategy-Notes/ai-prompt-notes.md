# Temp AI Prompts

## Prompt #1

Review BiobuzzTeleOp.java, provide recommendations on what to change to fix extreme latency problems we are seeing from the drive controls.  The goal of this class is to provide highly accurate control of the robot from gamepad1 while a second operator controls and exercises all of the subsystems in parallel.

## Prompt #2

Update BiobuzzTeleOp.java to have support for both Field Centric drive mode utilizing pedro pathing access to our pinpoint odometry system and Robot Centric Drive mode.  The gamepad1 operator should be able to easily switch between the two modes as needed and should be able to manually reset the heading to zero for field centric in case of a heading drift issue.  The robot should default to robot centric mode.

## Prompt #3

Move all of the telemetry messaging logic in BiobuzzTeleOp.java into a new procedure called PrintTelemetry() that is only called at a 5 Hz rate.

## Prompt #4

Move all Biobuzz package ColorSensor accessors into a new procedure called UpdateColorSensors() in the TransferSubsystem, combining the transfer color sensors and the intake subsystem's color sensors.  The only direct query of the physical color sensors should be done in this new procedure.  The color sensor data should be stored in local variables in the TransferSubsystem class.  All other classes will obtain access through the stored variables in TransferSubsystem without causing direct hardware access calls.  BiobuzzTeleOp, TransferTestOpMode, and IntakeAndTransferTestOpMode should all be updated to call this new UpdateColorSensor procedure at a 10 Hz rate.  Be sure to also move any config data for color sensors from the IntakeConfig class to the TransferConfig class.

## Prompt #5

Convert BiobuzzTeleOp.java to an iterative opMode instead of a LinearOpMode to minimize gamepad latency, minimize drive motor control latency, and minimize heading drift while still being able to access the color sensors and control the subsystems with minimal interference.  Enable bulk caching to minimize read calls on the hardware.  Rate-limit vision heading drift correction.  Move I2C color sensor updates to a dedicated asynchronous background thread. Eliminate object allocations within loops to prevent garbage collection hits.  Disable camera streaming during match play.  Minimize string buffer heap allocations where possible.  Reassess the BiobuzzTeleOp.java class after the changes and propose if there are other recommended changes needed to achieve minimal latency.

