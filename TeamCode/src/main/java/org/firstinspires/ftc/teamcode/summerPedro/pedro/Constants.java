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
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21335877164521483);
                Controller secondaryTranslationalForward = Controller.proportional(0.07883036507153075);
                Controller primaryTranslationalLateral = Controller.proportional(0.4778744465421269);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17656183895693353);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017277215412101126));
                c.brake.set(Controller.proportionalFeedforward(0.014685633100285957));

                c.headingFeedback.set(Controller.proportional(4.269078610812425));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04533786264190477, 0.012618056891091134));

                c.linearBrakeCoefficients.set(Matrix.diag(0.02735012787005562, 0.03073534403197522));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002499653283811774, 0.0028040054944306297));

                c.maxAchievableForwardVelocity.set(59.26715591770902);
                c.maxAchievableStrafeVelocity.set(44.27516776904426);
                c.naturalForwardDeceleration.set(38.28641426147586);
                c.naturalStrafeDeceleration.set(70.82996799545876);
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