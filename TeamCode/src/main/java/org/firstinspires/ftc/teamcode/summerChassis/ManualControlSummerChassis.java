package org.firstinspires.ftc.teamcode.summerChassis;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.summerChassis.SummerChassis;

@TeleOp(name = "1 Manual Control Summer Chassis")
@Disabled
public class ManualControlSummerChassis extends LinearOpMode {

    double ticks = 103.6;


    public void runOpMode() {


        SummerChassis theRobot;
        theRobot = new SummerChassis(true, true, this);
        theRobot.Init(this.hardwareMap);





        telemetry.update();
        waitForStart();
        resetRuntime();

        try {
            while (opModeIsActive()) {
                theRobot.driveControlsFieldCentric(); // do the field centric driving

                if (gamepad1.triangle) {
                    theRobot.imu.resetYaw();
                }

                telemetry.update();
            } // end while (opModeIsActive())


        } catch (Exception e) {

            // throw the exception higher for other handlers to run
            throw e;
        }

    }



}
