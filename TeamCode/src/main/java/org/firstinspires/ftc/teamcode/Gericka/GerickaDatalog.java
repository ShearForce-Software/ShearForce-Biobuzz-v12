package org.firstinspires.ftc.teamcode.Gericka;

import android.icu.text.SimpleDateFormat;

import java.io.FileWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Date;
import java.util.Locale;

public class GerickaDatalog {
    private FileWriter writer;
    private StringBuffer lineBuffer;
    private long timeBase;

    private boolean imuOffsetPopulated = false;
    private double imuOffset = 0.0;

    public Field resetPositionCount = new Field();
    public Field imuHeading = new Field();
    public Field correctedIMU_Heading = new Field();
    public Field imuDifference = new Field();
    public Field roadrunnerX = new Field();
    public Field roadrunnerY = new Field();
    public Field roadrunnerHeading = new Field();
    public Field distanceToRedReset = new Field();
    public Field distanceToBlueReset = new Field();
    public Field distanceToTarget = new Field();
    public Field LaunchRampPosition = new Field();
    public Field shooterRPM = new Field();
    public Field shooterTargetRPM = new Field();
    public Field YawScalar = new Field();
    public Field turretTicks = new Field();
    public Field turrentAngle = new Field();

    public GerickaDatalog(String filenamePrefix) {
        File dir = new File("/sdcard/FIRST/Datalogs");
        if (!dir.exists()) dir.mkdirs();

        // automatically delete any files that are more than 24 hours old
        deleteOldFiles(dir, 24);

        // create a new log file
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());
        File file = new File(dir, filenamePrefix + timestamp + ".csv");

        this.lineBuffer = new StringBuffer(256);
        this.timeBase = System.currentTimeMillis();
        try{
            this.writer = new FileWriter(file, false);
            writeHeader();
        } catch (IOException e) {
            throw new RuntimeException("Gericka Datalogger stream failed to open", e);
        }

        // initialize all of the fields
        resetPositionCount.set(0);
        roadrunnerX.set(0.0);
        roadrunnerY.set(0.0);
        roadrunnerHeading.set(0.0);
        imuDifference.set(0.0);
        correctedIMU_Heading.set(0.0);
        distanceToRedReset.set(0.0);
        distanceToBlueReset.set(0.0);
        distanceToTarget.set(0.0);
        LaunchRampPosition.set(0.0);
        shooterRPM.set(0.0);
        shooterTargetRPM.set(0.0);
        YawScalar.set(0.0);
        turretTicks.set(0.0);
        turrentAngle.set(0.0);
        imuHeading.set(0.0);
    }

    private void deleteOldFiles(File dir, int hoursThreshold) {
        // clean up and remove old log files to keep from filling up storage space

        long currentTime = System.currentTimeMillis();
        long maxAgeMs = currentTime - ((long)hoursThreshold * 60 * 60 * 1000);

        if (dir.exists() && dir.isDirectory()) {
            // get the list of files in this storage directory
            File[] files = dir.listFiles();
            if (files != null) {
                // loop through the list of files
                for ( File old_file : files) {
                    if (old_file.isFile()) {
                        long fileAgeMs = currentTime - maxAgeMs;
                        // if the file is old
                        if (old_file.lastModified() < maxAgeMs) {
                            // delete the old file
                            old_file.delete();
                        }
                    }
                }
            }
        }

    }
    private void writeHeader() throws IOException{
        writer.append("time(sec), resetCount" +
                ", imuHeading(deg), correctedIMU_Heading, roadrunner-heading(deg), imu-vs-rr-diff" +
                ", roadrunner-x, roadrunner-y, distance-to-RED-reset, distance-to-BLUE-reset" +
                ", distanceToTarget(in), LaunchRampPosition, shooterRPM, shooterTargetRPM" +
                ", YawScalar" +
                ", turretTicks, turrentAngle(deg) \n");
        writer.flush();
    }

    public void writeLine() {

        if (!imuOffsetPopulated) {
            try {
                imuOffset = Double.parseDouble(roadrunnerHeading.val);
                imuOffsetPopulated = true;
           } catch (NumberFormatException ignored) { }
        }

        try {
            // calculate the corrected IMU heading
            double rawIMU = Double.parseDouble(imuHeading.val);
            double correctedIMU = ((rawIMU + imuOffset + 180.0) - (360.0 * Math.floor((rawIMU + imuOffset + 180.0) / 360))) - 180.0;
            correctedIMU_Heading.set(correctedIMU);

            // calculate the difference between the roadrunner heading and the IMU heading
            double rrHeading = Double.parseDouble(roadrunnerHeading.val);
            double calculatedIMU_Diff = rrHeading - correctedIMU;
            imuDifference.set(calculatedIMU_Diff);

            // calculate the distance to the red reset position (62.785, -62.66) and the blue reset position (62.785, -62.66)
            double x = Double.parseDouble(roadrunnerX.val);
            double y = Double.parseDouble(roadrunnerY.val);
            double calculatedDistanceToRedReset = Math.sqrt(((x - 62.785) * (x - 62.785)) + ((y - (-62.66)) * (y - (-62.66))) );
            double calculatedDistanceToBlueReset = Math.sqrt(((x - 62.785) * (x - 62.785)) + ((y - (62.66)) * (y - (62.66))) );
            distanceToRedReset.set(calculatedDistanceToRedReset);
            distanceToBlueReset.set(calculatedDistanceToBlueReset);

        } catch (NumberFormatException ignored) { }


        try{
            lineBuffer.setLength(0);
            lineBuffer.append((System.currentTimeMillis() - timeBase)/1000.0).append(",");
            lineBuffer.append(resetPositionCount.val).append(",");

            lineBuffer.append(imuHeading.val).append(",");
            lineBuffer.append(correctedIMU_Heading.val).append(",");
            lineBuffer.append(roadrunnerHeading.val).append(",");
            lineBuffer.append(imuDifference.val).append(",");

            lineBuffer.append(roadrunnerX.val).append(",");
            lineBuffer.append(roadrunnerY.val).append(",");
            lineBuffer.append(distanceToRedReset.val).append(",");
            lineBuffer.append(distanceToBlueReset.val).append(",");

            lineBuffer.append(distanceToTarget.val).append(",");
            lineBuffer.append(LaunchRampPosition.val).append(",");
            lineBuffer.append(shooterRPM.val).append(",");
            lineBuffer.append(shooterTargetRPM.val).append(",");

            lineBuffer.append(YawScalar.val).append(",");

            lineBuffer.append(turretTicks.val).append(",");
            lineBuffer.append(turrentAngle.val).append("\n");

            writer.append(lineBuffer.toString());
        } catch (IOException ignored) {}
    }

    public void close() {
        try{
            if (writer != null) {
              writer.flush();
              writer.close();
            }
        } catch (IOException ignored) {}
    }

    public static class Field {
        public String val = "0";
        public void set(Object obj){
            this.val = String.valueOf(obj);
        }
    }
}
