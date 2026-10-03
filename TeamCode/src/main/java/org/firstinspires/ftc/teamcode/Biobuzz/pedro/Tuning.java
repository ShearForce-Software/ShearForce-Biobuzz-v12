package org.firstinspires.ftc.teamcode.Biobuzz.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Biobuzz.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.Biobuzz.pedro.procedures.Tests;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure biobuzz_mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure biobuzz_pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure biobuzz_tests() {

        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                (hardwareMap -> new PinpointLocalizer(hardwareMap,
                        Constants.localizerConfig)), null);
    }

    @Tuner
    public static Procedure biobuzz_foresightTuner() {
        return new ForesightTuner(
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }
}
