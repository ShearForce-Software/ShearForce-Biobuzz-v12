package org.firstinspires.ftc.teamcode.summerChassis;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class PedroPath_test {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(40, 10, 90);
    private final Pose point1 = poseFactory.of(40, 35, 90);
    private final Pose right = poseFactory.of(80, 35, 90);
    private final Pose point3 = poseFactory.of(80, 10, 90);
    private final Pose point4 = poseFactory.of(40, 10, 90);

    public Path forward() {
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path right() {
        return Paths.line(point1, right).linear(point1, right);
    }

    public Path back() {
        return Paths.line(right, point3).linear(right, point3);
    }

    public Path left() {
        return Paths.line(point3, point4).linear(point3, point4);
    }
}
