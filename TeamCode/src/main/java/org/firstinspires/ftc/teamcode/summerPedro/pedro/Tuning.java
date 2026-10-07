package org.firstinspires.ftc.teamcode.summerPedro.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.summerPedro.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.summerPedro.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.summerPedro.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.summerPedro.pedro.procedures.Tests;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure summerPedro_mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure summerPedro_pinpointTuner() {
        return new PinpointTuner();
    }

    /*@Tuner
    public static Procedure summerPedro_tests() {

        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                (hardwareMap -> new PinpointLocalizer(hardwareMap,
                        Constants.localizerConfig)), null);
    }*/

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)),
                () -> new Foresight(Constants.foresightConfig));
    }

    @Tuner
    public static Procedure summerPedro_foresightTuner() {
        return new ForesightTuner(
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }
}
