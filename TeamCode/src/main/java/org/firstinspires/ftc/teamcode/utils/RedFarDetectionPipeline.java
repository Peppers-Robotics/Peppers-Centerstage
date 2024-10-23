package org.firstinspires.ftc.teamcode.utils;

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
public class RedFarDetectionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    Telemetry telemetry;
    public enum Location{
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location = Location.LEFT;
    public boolean isBlue = false;

    public static int rightRectTopX = 300 , rightRectTopY = 100;
    public static int rightRectBottomX = 350 , rightRectBottomY = 160;

    public static int middleRectTopX = 140 ,middleRectTopY = 100;
    public static int middleRectBottomX = 100 ,middleRectBottomY = 160;

    public static int lowH = 100 ,lowS = 40, lowV = 30;
    public static int highH = 140, highS = 255, highV = 255;

    public static double tseThreshold = 0.12;

    public RedFarDetectionPipeline(Telemetry telemetry, boolean b) {this.telemetry = telemetry; isBlue = b;}

    @Override
    public Mat processFrame(Mat input){

        Rect RIGHT_ROI = new Rect(
                new Point(rightRectTopX, rightRectTopY),
                new Point(rightRectBottomX, rightRectBottomY));
        Rect MIDDLE_ROI = new Rect(
                new Point(middleRectTopX, middleRectTopY),
                new Point(middleRectBottomX, middleRectBottomY));

        if(isBlue)
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        else
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);
        Scalar lowHSV = new Scalar(lowH ,lowS, lowV);
        Scalar highHSV = new Scalar(highH ,highS, highV);
        Core.inRange(mat, lowHSV, highHSV ,mat);

        Mat left = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double rightValue = Core.sumElems(left).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        middle.release();

        boolean tseRight = rightValue > tseThreshold;
        boolean tseMiddle = middleValue > tseThreshold;

        if(tseRight) {
            location = Location.MIDDLE;
            telemetry.addData("pixel_location: ", "middle");
        }
        else if(tseMiddle){
            location = Location.LEFT;
            telemetry.addData("pixel_location: ", "left");
        }
        else{
            location = Location.RIGHT;
            telemetry.addData("pixel_location: ", "right");
        }
        telemetry.update();
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

        Scalar colorFound = new Scalar(255,0,0);
        Scalar colorNotFound = new Scalar(0,255,0);

        Imgproc.rectangle(mat,RIGHT_ROI,location == Location.RIGHT? colorFound:colorNotFound);
        Imgproc.rectangle(mat,MIDDLE_ROI,location == Location.MIDDLE? colorFound:colorNotFound);

        return mat;

    }
    public Location getLocation(){
        return location;
    }
    public void release(){
        mat.release();
    }
}