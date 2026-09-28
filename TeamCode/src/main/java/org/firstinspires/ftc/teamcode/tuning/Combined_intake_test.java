package org.firstinspires.ftc.teamcode.tuning;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Combined intake test", group = "TEST")
public class Combined_intake_test extends LinearOpMode {

    private static final String LEFT_NAME = "intakeLeft";
    private static final String RIGHT_NAME = "intakeRight";

    Servo servo;
    static final double INCREMENT = 0.05;
    static final double MAX_POS = 0.7;
    static final double MIN_POS = 0.0;


    private static final double INTAKE_POWER = 0.80;

    private DcMotorEx left, right;
    private DcMotorEx motor3, motor4;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotorEx.class, LEFT_NAME);
        right = hardwareMap.get(DcMotorEx.class, RIGHT_NAME);
        servo = hardwareMap.get(Servo.class, "servo-port2");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4");

        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // keep both FORWARD; we’ll use opposite signs for opposite spin
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);

        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Use y sticks on gamepad 1 and 2");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            double position = 0.0;

            // Scan servo till stop pressed.
            while (opModeIsActive()) {

                if (gamepad2.triangle) {
                    position = MAX_POS;
                } else if (gamepad2.cross) {
                    position = MIN_POS;
                } else if (gamepad2.circle) {
                    position = position + INCREMENT;
                    if (position > MAX_POS) {
                        position = MAX_POS;
                    }
                } else if (gamepad2.square) {
                    position = position - INCREMENT;
                    if (position < MIN_POS) {
                        position = MIN_POS;
                    }
                }
                servo.setPosition(position);

                left.setPower(gamepad1.left_stick_y);
                right.setPower(gamepad1.right_stick_y);// opposite direction

                motor3.setPower(gamepad2.left_stick_y);
                motor4.setPower(gamepad2.right_stick_y);
                // (do nothing if neither is pressed — power stays as-is)

                telemetry.addData("Motor 1", left.getPower());
                telemetry.addData("Motor 2", right.getPower());

                telemetry.addData("Motor 3", motor3.getPower());
                telemetry.addData("Motor 4", motor4.getPower());

                telemetry.addData("Servo Position", "%5.2f", position);
                telemetry.addData(">", "Servo plugged into control hub port 2.");
                telemetry.addData(">", "Servo is operated by gamepad 2.");
                telemetry.addData(">", "Press triangle to increase position to max.");
                telemetry.addData(">", "Press X to decrease position to min.");
                telemetry.addData(">", "Press circle to increase by increments.");
                telemetry.addData(">", "Press square to decrease by increments.");
                telemetry.update();
            }
        }
    }
}
