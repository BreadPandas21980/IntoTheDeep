package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RedAlliancePipeline extends OpenCvPipeline {
    //2
    public static int lowH =  0;
    public static int lowS = 70;
    //64
    public static int  lowV = 125;
    //352
    public static int highH = 100;
    public static int highS = 255;
    public static int highV = 255;
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MID,
        NOT_FOUND
    }
    public static Location location;

    /*
    static final Rect LEFT_ROI = new Rect(
            new Point(15, 230),
            new Point(90, 360)
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




    public RedAlliancePipeline(Telemetry t) {
        telemetry = t;
    }
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Scalar lowHSV = new Scalar(2, 36, 64);
        //Scalar highHSV = new Scalar(352, 255, 255);
        Scalar lowHSV = new Scalar(lowH, lowS, lowV);
        Scalar highHSV = new Scalar(highH, highS, highV);

        Core.inRange(mat, lowHSV, highHSV, mat);
        //Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);
       // double leftValue = (int) Core.mean(left).val[0];
        //double rightValue = (int) Core.mean(right).val[0];
        //double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        //left.release();
        mid.release();
        right.release();

        //telemetry.addData("Left raw value", (int)Core.sumElems(left).val[0]);
        //telemetry.addData("Right raw value", (int)Core.sumElems(right).val[0]);
        //telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        //boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD && leftValue < 0.45;
        boolean stoneMid = midValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        //boolean stoneRight = false;
        if(stoneMid) {
            location = Location.MID;
            telemetry.addData("Mid", "yayyyy!");
        }
        else if (stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Right", "ok!");
        } else {
            location = Location.LEFT;
            telemetry.addData("Left", "Left!");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar (255, 0, 0);
        Scalar colorSkystone = new Scalar (0, 255, 0);

        //Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MID_ROI, location == Location.MID? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;

    }

    public Location getLocation() {
        return location;
    }
}