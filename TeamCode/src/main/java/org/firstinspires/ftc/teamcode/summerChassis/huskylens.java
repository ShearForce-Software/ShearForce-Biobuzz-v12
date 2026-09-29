package org.firstinspires.ftc.teamcode.summerChassis;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HL_TagNumbers_All", group="Test")
public class huskylens extends LinearOpMode {


    private static final int IMG_W = 320;



    private static final int IMG_H = 240;
    private static final int CX = IMG_W / 2;   // 160




    private static final int CY = IMG_H / 2;   // 120


    private static final double TAG_HEIGHT_METERS = 0.10;


    private static final double FOCAL_PX = 520.0;



    private HuskyLens hl;

    @Override
    public void runOpMode() {

        hl = hardwareMap.get(HuskyLens.class, "huskylens");


        hl.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.setMsTransmissionInterval(50);




        telemetry.addLine("HuskyLens ready. Looking for AprilTags...");
        telemetry.update();

        waitForStart();

        if (!hl.knock()) {
           telemetry.addData(">>", "Problem communicating with " + hl.getDeviceName());
        } else {
           telemetry.addData(">>", "knock failed");
        }
        hl.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            telemetry.update();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = hl.blocks();



            if (blocks == null || blocks.length == 0) {
                telemetry.addLine("No tag seen");




                telemetry.update();
                continue;
            }

            telemetry.addData("blocks", blocks.length);

            // Report ALL detections
            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block b = blocks[i];

                int id     = b.id;       // 0 if not "learned" on device
                int x      = b.x;        // center x (px)
                int y      = b.y;        // center y (px)
                int left   = b.left;     // bbox left (px)
                int top    = b.top;      // bbox top (px)
                int width  = b.width;    // bbox width (px)
                int height = b.height;   // bbox height (px)

                int errX = x - CX;       // + right of center; - left
                int errY = y - CY;       // + below center;  - above
                int area = width * height;
                double aspect = (height == 0) ? 0.0 : (width * 1.0 / height);




                double distanceMeters = (height > 0)
                        ? (FOCAL_PX * TAG_HEIGHT_METERS) / height


                        : Double.POSITIVE_INFINITY;


                telemetry.addData("distance ~", "%.3f m (raw)", distanceMeters);
                telemetry.addLine(String.format("-- block[%d] --", i));
                telemetry.addData("ID", id);
                telemetry.addData("center(x,y)", "(%d, %d)", x, y);
                telemetry.addData("bbox[left,top]", "(%d, %d)", left, top);
                telemetry.addData("bbox[w,h]", "(%d, %d)", width, height);
                telemetry.addData("err(x,y from %d,%d)", "(%d, %d)", CX, CY, errX, errY);
                telemetry.addData("area, aspect", "%d, %.3f", area, aspect);
            }

            telemetry.update();
        }
    }
}
