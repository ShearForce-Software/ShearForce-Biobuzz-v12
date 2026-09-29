package org.firstinspires.ftc.teamcode.summerChassis;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="Summer Chassis Auto Alt", preselectTeleOp = "1 Manual Control Summer Chassis")
@Disabled
public class SummerChassisAutoRoutineAlt extends LinearOpMode {
    SummerChassis control = new SummerChassis(true, false,this);
    MecanumDrive_summerChassis drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose;
    Action FloorTraj;
    Action DriveToStack;
    Action BoardTraj2;
    Action Park;
    Action DriveBackToStack;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;
    double stackY = -36.0;
    double stackX = -59.0;

    public void runOpMode(){
        startPose = new Pose2d(-36,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(75.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(5); 
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

        /* Initialize the Robot */
        drive = new MecanumDrive_summerChassis(hardwareMap, startPose);
        control.Init(hardwareMap);
        telemetry.update();
        control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        while(!isStarted()){
            telemetry.update();//make decisions
            BlueRightPurplePixelDecision();
        }
        resetRuntime();
        control.autoTimeLeft = 0.0;

        // Create the floor to Stack trajectory
        // if(control.autoPosition==1) {
            DriveToStack = drive.actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(-39, -50), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(stackX+5, stackY, Math.toRadians(180)), Math.toRadians(0), null, slowDownAccelerationConstraint)
                    .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackX-2, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                    .build();
        //}

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        /* Drive to Floor Position */
                        new ParallelAction(
                                lockPixels(),
                                FloorTraj,
                                new SequentialAction(
                                        new SleepAction(.25),
                                        armRotationsPurplePixelDelivery(),
                                        wristRotationsPurplePixelDelivery(),
                                        new SleepAction(.275)
                                )
                        ),
                        /* Deliver the Purple Pixel */
                        new SequentialAction(
                                releasePurplePixel(),
                                new SleepAction(.15),
                                clearanceAfterPurpleDelivery()
                        ),
                        /* Drive to the stack of white pixels */
                        new ParallelAction(
                                resetArm(),
                                servoIntake(),
                                DriveToStack
                        )
                )
        );

        /* Intake White Pixel(s) from the stack */
        drive.updatePoseEstimate();

        /* Drive to the board while moving arm up to scoring position after crossing the half-way point */
        BlueBoardDecision(); // updates BoardTraj2
        Actions.runBlocking(new SequentialAction(
                positionArmWristToGrab(),
                new SleepAction(.25),
                autograb(),
                new SleepAction(.25),
                new ParallelAction(
                        new SequentialAction(
                                pickUpWhitePixels(),
                                new SleepAction(.3),
                                servoOuttake()
                        ),

                        new SequentialAction(
                                new SleepAction(.15),
                                BoardTraj2
                        ),
                        new SequentialAction(
                                halfwayTrigger1_raiseSlidesToAutoHeight(),
                                new SleepAction(.15),
                                halfwayTrigger2_moveArmToBoardDeliverPos(),
                                new SleepAction(.15),
                                halfwayTrigger3_moveWristToBoardDeliverPos()
                                )
                        )
                )
        );

        /* Use AprilTags to Align Perfectly to the Board */

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(drive.localizer.getPose().position.x + .5, drive.localizer.getPose().position.y), Math.toRadians(180))
                        .build());

        /* release pixels on the board using the distance sensor to know when to stop */


        /* BACK UP FROM BOARD slightly so that the pixels fall off cleanly */
        drive.updatePoseEstimate();
        Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                .lineToX(drive.localizer.getPose().position.x-1.5)
                .build());

        // **********************************************************
        // ******    Begin Logic to get an extra 2 White Pixels *****
        // **********************************************************
        drive.updatePoseEstimate();
        DriveBackToStack = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(30, -58), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(12, -58), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-12, -58), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-36, -58), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(-52, -58), Math.toRadians(180), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(stackX+5, stackY), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackX, stackY), Math.toRadians(180))
                .build();

        //drive.useExtraCorrectionLogic = true;
        Actions.runBlocking(
                new ParallelAction(
                        DriveBackToStack,
                        new SequentialAction(
                                resetArm(),
                                new SleepAction(.15),
                                slidesDown()
                        ),
                        servoIntake()
                )
        );
        //drive.useExtraCorrectionLogic = false;


        /* Use camera to make a minor adjustment to position if needed */
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(stackX-2,drive.localizer.getPose().position.y), Math.toRadians(180), slowDownVelocityConstraint)
                        .build()
        );
        drive.updatePoseEstimate();


        //grab 2 more white pixels
        drive.updatePoseEstimate();

        //drive to position 3
        BoardTraj2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-36, -58), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-12, -58), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(12, -58), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(30, -58), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(deliverToBoardPose.position.x, deliverToBoardPose.position.y), Math.toRadians(180))
                .build();

        Actions.runBlocking(new SequentialAction(
                        positionArmWristToGrab(),
                        new SleepAction(.25),
                        autograb(),
                        new SleepAction(.25),
                        new ParallelAction(
                                new SequentialAction(
                                        pickUpWhitePixels(),
                                        new SleepAction(.3),
                                        servoOuttake()
                                        ),
                                new SequentialAction(
                                        new SleepAction(.15),
                                        BoardTraj2
                                ),
                                new SequentialAction(
                                        halfwayTrigger1_raiseSlidesToAutoHeight(),
                                        new SleepAction(.15),
                                        halfwayTrigger2_moveArmToBoardDeliverPos(),
                                        new SleepAction(.15),
                                        halfwayTrigger3_moveWristToBoardDeliverPos()
                                )
                        )
                )
        );

        //deliver two white pixels
        drive.updatePoseEstimate();

        /* Park the Robot, and Reset the Arm and slides */
        Park = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(45, slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(46, 27), Math.toRadians(270))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        Park,
                        new SequentialAction(
                                new SleepAction(.1),
                                resetArm(),
                                new SleepAction(.15),
                                slidesDown()
                        ),
                        servoStop()
                )
        );

        control.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", control.autoTimeLeft);
        telemetry.update();
    }

    public void BlueBoardDecision() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(46,-27,Math.toRadians(180));
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(46,-39,Math.toRadians(180));
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(46,-33,Math.toRadians(180));
        }
        BoardTraj2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(stackX + 1.0, stackY), Math.toRadians(180), slowDownVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-36, -58), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(-12, -58), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(12, -58), speedUpVelocityConstraint)
                .strafeToConstantHeading(new Vector2d(30, -58), speedUpVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(deliverToBoardPose.position.x, -41.5), Math.toRadians(180))
                .build();
    }
    public void BlueRightPurplePixelDecision() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-33, -30, Math.toRadians(0));
            FloorTraj = drive.actionBuilder(startPose)
                    //.splineToLinearHeading(new Pose2d(-38.5, 35.5, Math.toRadians(270)), Math.toRadians(270))
                    //.strafeToLinearHeading(new Vector2d(-31, 34), Math.toRadians(180))
                    //.strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(180))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .build();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-36, -34.5, Math.toRadians(180));
            FloorTraj = drive.actionBuilder(startPose)
                    //.splineToLinearHeading(new Pose2d(-38.5, 33, Math.toRadians(270)), Math.toRadians(270))
                    //.splineToLinearHeading (deliverToFloorPose, Math.toRadians(315))
                    .splineToLinearHeading(new Pose2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y, Math.toRadians(180)), Math.toRadians(0), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-36, -37.5, Math.toRadians(270));
            FloorTraj = drive.actionBuilder(startPose)
                    .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(270), speedUpVelocityConstraint, slowDownAccelerationConstraint)
                    .build();
        }
    }

    public Action lockPixels(){return new LockPixels();}
    public class LockPixels implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action dropOnLine(){return new DropOnLine();}
    public class DropOnLine implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action resetArm(){return new ResetArm();}
    public class ResetArm implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("ResetArm", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesDown(){return new SlidesDown();}
    public class SlidesDown implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("Slides Down", 0);
            boolean slidesAllDown = false;
            return slidesAllDown;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action positionArmWristToGrab(){return new AutoGrab1();}
    public class AutoGrab1 implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Stop the spinners and
                // Move arm and wrist down to grab the pixels
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action pickUpWhitePixels(){return new AutoGrab2();}
    public class AutoGrab2 implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action autograb(){return new Grab();}
    public class Grab implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoOuttake(){return new ServoOuttake();}
    public class ServoOuttake implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("ServoOuttake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoIntake(){return new ServoIntake();}
    public class ServoIntake implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger1_raiseSlidesToAutoHeight(){return new HalfwayTrigger1();}
    public class HalfwayTrigger1 implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.localizer.getPose().position.x >= 12) {
                moveArm = true;
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger2_moveArmToBoardDeliverPos(){return new HalfwayTrigger2();}
    public class HalfwayTrigger2 implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.localizer.getPose().position.x >= 12) {
                moveArm = true;
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger3_moveWristToBoardDeliverPos(){return new HalfwayTrigger3();}
    public class HalfwayTrigger3 implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            //drive.updatePoseEstimate();
            if (drive.localizer.getPose().position.x >= 12) {
                moveArm = true;
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoStop(){return new ServoStop();}
    public class ServoStop implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;
            }
    }

    public Action prepareToDropPurplePixel() {
        return new PrepareToDropPurplePixel();
    }

    public class PrepareToDropPurplePixel implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("prepare to drop purple pixel on line", 0);
            return false;
        }
    }

    public Action armRotationsPurplePixelDelivery() {
        return new ArmRotationsPurplePixelDelivery();
    }

    public class ArmRotationsPurplePixelDelivery implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("Rotate Arm to deliver purple pixel on line", 0);
            return false;
        }
    }
    public Action wristRotationsPurplePixelDelivery() {
        return new WristRotationsPurplePixelDelivery();
    }

    public class WristRotationsPurplePixelDelivery implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("Adjust wrist to deliver the purple pixel on line", 0);
            return false;
        }
    }


    public Action releasePurplePixel() {
        return new ReleasePurplePixel();
    }

    public class ReleasePurplePixel implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("Release purple pixel on line", 0);
            return false;
        }
    }

    public Action clearanceAfterPurpleDelivery() {
        return new ClearanceAfterPurpleDelivery();
    }

    public class ClearanceAfterPurpleDelivery implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            packet.put("Clearance of arm mechanism after purple pixel delivery", 0);
            return false;
        }
    }
}

