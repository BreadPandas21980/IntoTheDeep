package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Config
public class BlueAlliancePipeline extends OpenCvPipeline {
    Telemetry telemetry;
    boolean seen = false;
    public static int lowH = 80;
    public static int lowS = 120;
    //64
    public static int  lowV = 0;
    //352
    public static int highH = 110;
    public static int highS = 255;
    public static int highV = 255;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MID,
        NOT_FOUND
    }
    public static Location location = Location.MID;


/*
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 250),
            new Point(50, 340)
    );

 */



    static final Rect MID_ROI = new Rect(
            new Point(325, 250),
            new Point(425, 340)
    );


    static final Rect RIGHT_ROI = new Rect(
            new Point(750, 250),
            new Point(800, 400)
    );




    public BlueAlliancePipeline(Telemetry t) {
        telemetry = t;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    static double PERCENT_COLOR_THRESHOLD = 0.3;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //Scalar lowHSV = new Scalar(2, 36, 64);
        //Scalar highHSV = new Scalar(352, 255, 255);
        Scalar lowHSV = new Scalar(lowH, lowS, lowV); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(highH, highS, highV
        ); // lenient higher bound HSV for yellow

        Core.inRange(mat, lowHSV, highHSV, mat);
        //Mat left = mat.submat(LEFT_ROI);
        //Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        if (Core.sumElems(mid).val[0] / MID_ROI.area() / 255 > PERCENT_COLOR_THRESHOLD &&
                Core.sumElems(mid).val[0] / MID_ROI.area() / 255 > Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255) {
            location = Location.MID;
            seen = true;
            telemetry.addData("Mid", "yayyyy!");
        }
        else if (Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255 > PERCENT_COLOR_THRESHOLD) {
            seen = true;
            location = Location.RIGHT;
            telemetry.addData("Right", "ok!");
        }
        if(seen == false) {
            location = Location.LEFT;
            telemetry.addData("Left", (Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255 > 0.52));
        }
        // double leftValue = (int) Core.mean(left).val[0];
        //double rightValue = (int) Core.mean(right).val[0];
        //double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        //double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        //left.release();
        //left.release();

        //telemetry.addData("Left raw value", (int)Core.sumElems(left).val[0]);
        //telemetry.addData("Right raw value", (int)Core.sumElems(right).val[0]);
        //telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        //telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Mid value: ", midValue);
        telemetry.addData("Right value: ", rightValue);

        //boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD && leftValue < 0.45;
        //boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMid = midValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;


        telemetry.update();

        mid.release();
        right.release();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar (255, 0, 0);
        Scalar colorSkystone = new Scalar (0, 255, 0);

        //Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        //Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;

    }

    public Location getLocation() {
        return location;
    }
}