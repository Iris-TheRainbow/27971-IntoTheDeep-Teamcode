package org.firstinspires.ftc.teamcode.commandbase;


import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class Vision implements VisionProcessor{

    public interface TeamColor{
        @NonNull
        String toString();
        Scalar getUpper();
        Scalar getLower();
        int getDrawColor();
    }
    public static class RedAlliance implements TeamColor{
        @NonNull
        @Override
        public String toString(){return "RED"; }
        public Scalar getUpper(){ return new Scalar(3, 255, 999, 0); }
        public Scalar getLower(){ return new Scalar(250, 160, 180, 0); }
        public int getDrawColor(){ return Color.RED; }
    }
    public static class BlueAlliance implements TeamColor{
        @NonNull
        @Override
        public String toString(){return "BLUE"; }
        public Scalar getUpper(){ return new Scalar(200, 240, 240, 0); }
        public Scalar getLower(){ return new Scalar(140, 40, 0, 0); }
        public int getDrawColor(){ return Color.BLUE; }
    }
    public static class NullAlliance implements TeamColor{
        @NonNull
        @Override
        public String toString(){return "NULL"; }
        public Scalar getUpper(){ return new Scalar(0, 0, 0, 0); }
        public Scalar getLower(){ return new Scalar(0, 0, 0, 0); }
        public int getDrawColor(){ return Color.BLACK; }
    }

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final Scalar upperTeamColor;
    private final Scalar lowerTeamColor;
    private final Scalar yellowUpper = new Scalar(50, 200, 255, 0);
    private final Scalar yellowLower = new Scalar(35, 70, 160, 0);
    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
    private final MatOfPoint2f mat2f = new MatOfPoint2f();
    private final Mat sel1 = new Mat(); // these facilitate capturing through 0
    private final Mat sel2 = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat working = new Mat();
    private final double maxArea = 9000;
    private final double minArea = 3500;
    private int width;
    private int height;
    private final TeamColor color;
    private double closestXInches;
    private double closestYInches;
    private double closestRotate;
    private final double pxToIn = .04;
    public Vision(TeamColor color){
        this.color = color;
        this.upperTeamColor = color.getUpper();
        this.lowerTeamColor = color.getLower();
    }

    public double getClosestXInches() {
        return closestXInches;
    }

    public double getClosestYInches() {
        return closestYInches;
    }
    public double getClosestRotate(){
        return closestRotate;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Core.flip(frame, frame, 0);
        Mat temp = new Mat();

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        ArrayList<RotatedRect> samples = new ArrayList<>();
        Imgproc.cvtColor(frame, working, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.cvtColor(frame, temp, Imgproc.COLOR_RGB2HSV_FULL);
        Imgproc.GaussianBlur(working, working, new Size(13, 13), 6);
        Imgproc.GaussianBlur(temp, temp, new Size(13,13), 6);
        contours.clear();
        if (upperTeamColor.val[0] < lowerTeamColor.val[0]) {
            Core.inRange(working, new Scalar(upperTeamColor.val[0], lowerTeamColor.val[1], lowerTeamColor.val[2]), new Scalar(0, upperTeamColor.val[1], upperTeamColor.val[2]), sel1);
            Core.inRange(working, new Scalar(0, lowerTeamColor.val[1], lowerTeamColor.val[2]), new Scalar(lowerTeamColor.val[0], upperTeamColor.val[1], upperTeamColor.val[2]), sel2);
            Core.bitwise_or(sel1, sel2, frame);
        } else {
            //filter team color
            Core.inRange(working, lowerTeamColor, upperTeamColor, frame);
        }

        //filter out yellow1
        if (!(this.color instanceof RedAlliance)) {
            Core.inRange(temp, yellowLower, yellowUpper, temp);
            Core.bitwise_or(temp, frame, frame);
        }
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour: contours){
            mat2f.release();
            contour.convertTo(mat2f, CvType.CV_32F);
            double area = Imgproc.contourArea(contour);
            if (area > minArea && area < maxArea) {
                samples.add(Imgproc.minAreaRect(mat2f));
            }
        }

        return samples;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(2);
        ArrayList<RotatedRect> samples = (ArrayList<RotatedRect>) userContext;
        RotatedRect closestSample = getClosestSample(samples);
        for (RotatedRect sample: samples){
            org.opencv.core.Point[] vertices = new Point[4];
            paint.setColor(android.graphics.Color.RED);
            if (sample.equals(closestSample)){
                paint.setColor(android.graphics.Color.GREEN);
            }
            sample.points(vertices);
            canvas.drawText("" + sample.angle, (float) sample.center.x, (float) sample.center.y, paint);
            for (int i = 0; i < 4; i++) {
                if (!(i==3)) {
                    canvas.drawLine((float) (scaleBmpPxToCanvasPx * vertices[i].x), (float) (scaleBmpPxToCanvasPx * vertices[i].y), (float) (scaleBmpPxToCanvasPx * vertices[i + 1].x), (float) (scaleBmpPxToCanvasPx * vertices[i + 1].y), paint);
                } else{
                    canvas.drawLine((float) (scaleBmpPxToCanvasPx * vertices[i].x), (float) (scaleBmpPxToCanvasPx * vertices[i].y), (float) (scaleBmpPxToCanvasPx * vertices[0].x), (float) (scaleBmpPxToCanvasPx * vertices[0].y), paint);
                }
            }
        }
        paint.setStrokeWidth(10);
        canvas.drawLine(scaleBmpPxToCanvasPx * width/2, scaleBmpPxToCanvasPx * height/2, scaleBmpPxToCanvasPx * width/2, (float)(scaleBmpPxToCanvasPx*height/2 + 1/pxToIn), paint);
    }
    public static double distance(double x1, double y1, double x2, double y2){
        return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
    }
    public RotatedRect getClosestSample(ArrayList<RotatedRect> samples){
        RotatedRect closest;
        try{
            closest = samples.get(0);
            for (RotatedRect sample: samples){
                if (distance(sample.center.x, sample.center.y, width/2.0, height/2.0) <  distance(closest.center.x, closest.center.y, width/2.0, height/2.0)){
                    closest = sample;
                }
            }
        } catch (Exception e){
            closest = new RotatedRect(new Point(0,0), new Size(0, 0), 0);
        }
        closestXInches = (closest.center.y - height / 2) * pxToIn;
        closestYInches = -((closest.center.x - width / 2) * pxToIn);
        if (closest.equals(new RotatedRect(new Point(0, 0), new Size(0, 0), 0))){
            closestXInches = 0;
            closestYInches = 0;
            closestRotate = .5;
        }
        return closest;
    }
}
