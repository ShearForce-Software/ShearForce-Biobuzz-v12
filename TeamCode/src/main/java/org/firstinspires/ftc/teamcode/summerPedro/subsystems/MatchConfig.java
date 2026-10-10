package org.firstinspires.ftc.teamcode.summerPedro.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

/**
 * Common Match Configuration class for the BioBuzz FTC season.
 * Manages alliance color, starting position, goal locations, goal states,
 * and persistent robot pose tracking across Autonomous and TeleOp transitions.
 * Annotated with @Config for interactive control via FTC Dashboard.
 */
@Config
public class MatchConfig {

    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum StartPosition {
        AUDIENCE,
        BACKSTAGE
    }

    // Match configuration settings
    public static AllianceColor allianceColor = AllianceColor.RED;
    public static StartPosition startPosition = StartPosition.AUDIENCE;

    // Goal status indicator (true = Audience Goal is UP, false = Audience Goal is DOWN)
    public static boolean audienceGoalUp = true;

    // Persistent robot pose stored at the end of Autonomous and restored at the start of TeleOp
    public static Pose currentPose = new Pose(0, 0, 0);

    // Center location Poses for RED and BLUE alliance goals (in Pedro field coordinates)
    public static Pose RED_AUDIENCE_GOAL = new Pose(-48.0, 48.0, Math.toRadians(0));
    public static Pose BLUE_AUDIENCE_GOAL = new Pose(48.0, 48.0, Math.toRadians(180));

    public static Pose RED_BACKSTAGE_GOAL = new Pose(-48.0, -48.0, Math.toRadians(0));
    public static Pose BLUE_BACKSTAGE_GOAL = new Pose(48.0, -48.0, Math.toRadians(180));

    // Private constructor prevents instantiation
    private MatchConfig() {}

    /**
     * Returns true if the currently selected alliance is RED.
     */
    public static boolean isRedAlliance() {
        return allianceColor == AllianceColor.RED;
    }

    /**
     * Returns true if the currently selected alliance is BLUE.
     */
    public static boolean isBlueAlliance() {
        return allianceColor == AllianceColor.BLUE;
    }

    /**
     * Returns true if the starting position is AUDIENCE.
     */
    public static boolean isAudienceStart() {
        return startPosition == StartPosition.AUDIENCE;
    }

    /**
     * Returns true if the Audience Goal is currently in the UP position.
     */
    public static boolean isAudienceGoalUp() {
        return audienceGoalUp;
    }

    /**
     * Returns the center location Pose of the Audience Goal based on the active Alliance Color.
     */
    public static Pose getAudienceGoalPose() {
        return isRedAlliance() ? RED_AUDIENCE_GOAL : BLUE_AUDIENCE_GOAL;
    }

    /**
     * Returns the center location Pose of the Backstage Goal based on the active Alliance Color.
     */
    public static Pose getBackstageGoalPose() {
        return isRedAlliance() ? RED_BACKSTAGE_GOAL : BLUE_BACKSTAGE_GOAL;
    }
}
