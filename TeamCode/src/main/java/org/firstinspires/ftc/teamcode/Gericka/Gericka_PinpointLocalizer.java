package org.firstinspires.ftc.teamcode.Gericka;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Localizer;

import java.util.Objects;

@Config
public final class Gericka_PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public Gericka_PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //double mmPerTick = inPerTick * 25.4;
        //driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        // Xpod is -84.0 MM from pinpoint computer, and -109.5 mm from center point of robot
        // Ypod is aligned with pinpoint computer in X, and -198.0 mm behind the center point of robot
        // before 8-20 yOffset was -198.0 xoffset was 109.5
        double xOffset = -102; //mm - X pod is to the right of pinpoint when viewed from the top
        double yOffset = -198; // mm - Y pod is -198 mm from center rotation point of chassis -- not sure this is correct value for pinpoint computer
        driver.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        // reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD; //TODO double check this value, initial calibration had FORWARD here, but Gericka_Hardware shows it as REVERSED

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        driver.setPosition(new Pose2D(DistanceUnit.INCH, initialPose.position.x, initialPose.position.y, AngleUnit.RADIANS,initialPose.heading.toDouble()));

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
