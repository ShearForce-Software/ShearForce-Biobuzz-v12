package org.firstinspires.ftc.teamcode.Biobuzz.pedro;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.summerPedro.pedro.Constants.foresightConfig;

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
import com.pedropathing.tuning.autotune.Procedure;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

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
        c.xPodOffset.set(-4.81446183572604);
        c.yPodOffset.set(-4.008915518212506);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3690919905455153);
                Controller secondaryTranslationalForward = Controller.proportional(0.1363696281869436);
                Controller primaryTranslationalLateral = Controller.proportional(0.5705442407955879);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21080085175108038);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013367924375225158));
                c.brake.set(Controller.proportionalFeedforward(0.011362735718941384));

                c.headingFeedback.set(Controller.proportional(6.995036971177747));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06773554493619002, 0.0039853864392787545));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08785962799743102, 0.06959282857513155));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001593463731013719, 0.0015659578683250549));

                c.maxAchievableForwardVelocity.set(73.03179046703305);
                c.maxAchievableStrafeVelocity.set(58.34236326235492);
                c.naturalForwardDeceleration.set(28.666189060162694);
                c.naturalStrafeDeceleration.set(66.92071365826644);
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