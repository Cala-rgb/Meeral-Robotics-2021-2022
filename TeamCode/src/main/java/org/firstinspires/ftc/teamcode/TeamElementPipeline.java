package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayDeque;

public class TeamElementPipeline extends OpenCvPipeline {
    public enum TeamElementPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile TeamElementPosition position = TeamElementPosition.LEFT;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70,260);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(270,260);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(475,260);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 40;

    private final ArrayDeque<Integer> lastMin1 = new ArrayDeque<>(), lastMin2 = new ArrayDeque<>(), lastMin3 = new ArrayDeque<>();

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region1_Cr, region2_Cr, region3_Cr;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    int avg1, avg2, avg3;

    private void addToLastValues() {
        lastMin1.addLast(avg1);
        lastMin2.addLast(avg2);
        lastMin3.addLast(avg3);
        if (lastMin1.size() >= 11) {
            lastMin1.removeFirst();
        }
        if (lastMin2.size() >= 11) {
            lastMin2.removeFirst();
        }
        if (lastMin3.size() >= 11) {
            lastMin3.removeFirst();
        }
    }

    private int getOverallMinAvarageRectangle() {
        // 1 - first square
        // 2 - second square
        // 3 - third square
        int overallAvg1 = 0, overallAvg2 = 0, overallAvg3 = 0;
        ArrayDeque<Integer> lastMin1_c = lastMin1.clone(), lastMin2_c = lastMin2.clone(), lastMin3_c = lastMin3.clone();
        while (!lastMin1_c.isEmpty()) {
            overallAvg1 += lastMin1_c.getFirst();
            lastMin1_c.removeFirst();
        }
        while (!lastMin2_c.isEmpty()) {
            overallAvg2 += lastMin2_c.getFirst();
            lastMin2_c.removeFirst();
        }
        while (!lastMin3_c.isEmpty()) {
            overallAvg3 += lastMin3_c.getFirst();
            lastMin3_c.removeFirst();
        }
        overallAvg1 = overallAvg1 / lastMin1.size();
        overallAvg2 = overallAvg2 / lastMin2.size();
        overallAvg3 = overallAvg3 / lastMin3.size();
        int min = Math.min(Math.min(overallAvg1, overallAvg2), overallAvg3);
        if (overallAvg1 == min) return 1;
        if (overallAvg2 == min) return 2;
        return 3;
    }

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);

        region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);
        avg1 = (int) Core.mean(region1_Cr).val[0];
        avg2 = (int) Core.mean(region2_Cr).val[0];
        avg3 = (int) Core.mean(region3_Cr).val[0];
        addToLastValues();

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        int result = getOverallMinAvarageRectangle();

        if(result == 1) // Was it from region 1?
        {
            position = TeamElementPosition.LEFT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(result == 2) // Was it from region 2?
        {
            position = TeamElementPosition.CENTER; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(result == 3) // Was it from region 3?
        {
            position = TeamElementPosition.RIGHT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        return input;
    }

    public TeamElementPosition getAnalysis()
    {
        return position;
    }
}
