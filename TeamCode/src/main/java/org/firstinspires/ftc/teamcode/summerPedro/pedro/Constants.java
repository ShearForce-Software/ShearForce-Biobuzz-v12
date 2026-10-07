package org.firstinspires.ftc.teamcode.summerPedro.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront_leftOdometry");
        c.frontRightName.set("rightFront_centerOdometry");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear_rightOdometry");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.914435799666276);
        c.yPodOffset.set(-2.1994952704962785);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19323925993344265);
                Controller secondaryTranslationalForward = Controller.proportional(0.07139674309728507);
                Controller primaryTranslationalLateral = Controller.proportional(0.44752905988481206);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16535002943072066);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016595331715246995));
                c.brake.set(Controller.proportionalFeedforward(0.014106031957959946));

                c.headingFeedback.set(Controller.proportional(4.088247060702795));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04867706321902048, 0.01287657328187877));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06645286329486476, 0.06038077448740835));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001501062732708966, 0.0016897226495431175));

                c.maxAchievableForwardVelocity.set(62.55610496407881);
                c.maxAchievableStrafeVelocity.set(46.69066723895943);
                c.naturalForwardDeceleration.set(32.60012276199946);
                c.naturalStrafeDeceleration.set(67.37881146943177);
            }
    );

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}