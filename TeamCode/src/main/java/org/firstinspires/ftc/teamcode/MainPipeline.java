package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class MainPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public static Scalar lowHSV = new Scalar(130, 80, 80);
    public static Scalar highHSV = new Scalar(255, 255, 255);
    public static Scalar strictLowHSV = new Scalar(40, 35, 110);
    public static Scalar strictHighHSV = new Scalar(255, 255, 255);

    public static int cannyThresh1 = 100;
    public static int cannyThresh2 = 200;
    double centerError;
    double frameCenter = 240;

    public MainPipeline(Telemetry t) {
        t = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) return input;

        Mat thresh = new Mat();

        Core.inRange(mat, lowHSV, highHSV, thresh);


        Mat masked = new Mat();

        Core.bitwise_and(mat, mat, masked, thresh);

        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();

        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();

        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();

        Core.bitwise_and(mat, mat, finalMask, scaledThresh);


        Mat edges = new Mat();

        Imgproc.Canny(finalMask, edges, cannyThresh1, cannyThresh2);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
//              double area = Imgproc.contourArea(contour);
            Point[] contourArray = contour.toArray();
            if (contourArray.length >= 15) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() >= 2500) {
//                telemetry.addData("contour length", contourArray.length);

                    Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 2);
                    Imgproc.circle(input, new Point(rect.x + rect.width / 2, rect.y + rect.height / 2), 1, new Scalar(255, 0, 0));
                }
            }
        }

        scaledThresh.release();
        scaledMask.release();
        mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();

        return input;

    }

    public double lockOn(double x, double w) {
        try {
            double centerX = x + w / 2;

            centerError = centerX - frameCenter;

            return centerError;
        } catch (Exception e) {
            centerError = 0;
        }

        return 0;
    }

    public double moveServo(double centerPos, double currPosition) {
        if (centerPos > 0) {
            telemetry.addData("left", (currPosition - centerPos) / 500);
            return (currPosition - centerPos) / 500;

        } else if (centerPos < 0) {
            telemetry.addData("right", (currPosition + centerPos) / 500);
            return (currPosition + centerPos) / 500;
        } else {
            return 0;
        }
    }

    public double getCenterError() {
        return centerError;
    }

}
