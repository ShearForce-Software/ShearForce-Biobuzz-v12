package org.firstinspires.ftc.teamcode.Gericka;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Disabled
@Autonomous(name = "Gericka Obelisk Reader", group = "Gericka")
public class ObeliskReadingTest extends LinearOpMode {



    private Gericka_Hardware theRobot;


    public enum Motif {
        GPP, PGP, PPG, UNKNOWN
    }

    private Motif detectedMotif = Motif.UNKNOWN;
    private int detectedTagId = -1;


    private Motif detectObeliskMotifOnce() {

        if (theRobot.getAprilTagVisible(21)) {
            detectedTagId = 21;
            return Motif.GPP;
        }
        if (theRobot.getAprilTagVisible(22)) {
            detectedTagId = 22;
            return Motif.PGP;
        }
        if (theRobot.getAprilTagVisible(23)) {
            detectedTagId = 23;
            return Motif.PPG;
        }

        // cannot see anythign
        detectedTagId = -1;
        return Motif.UNKNOWN;
    }

    private String motifToCoreString(Motif m) {
        switch (m) {
            case GPP: return "GPP";
            case PGP: return "PGP";
            case PPG: return "PPG";
            default:  return "UNKNOWN";
        }
    }

    private String expandToNine(String core) {
        if (core.equals("UNKNOWN")) return "UNKNOWN";
        // Repeat the 3-letter motif 3 times -> 9 indices for the ramp
        return core + core + core;
    }

    @Override
    public void runOpMode() {
       //change alliance code color later
        theRobot = new Gericka_Hardware(false, true, this);
        theRobot.Init(hardwareMap, "BLUE");

        // Use the existing webcam + AprilTag setup from Gericka_Hardware
        theRobot.WebcamInit(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        detectedMotif = Motif.UNKNOWN;
        detectedTagId = -1;


        while (!isStarted() && !isStopRequested()) {

            Motif current = detectObeliskMotifOnce();
            if (current != Motif.UNKNOWN) {
                detectedMotif = current;
            } // Set the motif to whatever is sees.

            String core = motifToCoreString(detectedMotif);
            String fullPattern = expandToNine(core);

            telemetry.addLine("=== Obelisk Reader INIT ===");
            telemetry.addData("Visible Obelisk Tag ID", detectedTagId); // Should be unknown at beginning
            telemetry.addData("Detected Motif (3)", core);
            telemetry.addData("Expanded Pattern (9)", fullPattern);
            telemetry.addLine("Looking for tag IDs 21, 22, 23...");

            //now we can put in th ein blackboard so other OpModes (autos, teleop) can read it
            blackboard.put("OBELISK_TAG_ID", detectedTagId);
            blackboard.put("OBELISK_MOTIF", core);
            blackboard.put("OBELISK_PATTERN_9", fullPattern);

            telemetry.update();
            sleep(20);
        }

        // Wait for official start
        waitForStart();

        //  Just hold the final result and keep showing it
        String finalCore = motifToCoreString(detectedMotif);
        String finalPattern = expandToNine(finalCore);

        while (opModeIsActive()) {
            Motif current = detectObeliskMotifOnce();
            if(current !=Motif.UNKNOWN){
                detectedMotif = current;
            }
            //String core = motifToCoreString(detectedMotif);
           // String fullPattern = expandToNine(core);
            finalCore = motifToCoreString(detectedMotif);
            finalPattern = expandToNine(finalCore);


            telemetry.addLine("=== Obelisk Reader RUNNING ===");
            telemetry.addData("Final Obelisk Tag ID", detectedTagId);
            telemetry.addData("Final Motif (3)", finalCore);
            telemetry.addData("Final Pattern (9)", finalPattern);
            telemetry.update();
            idle();
        }
    }
}
