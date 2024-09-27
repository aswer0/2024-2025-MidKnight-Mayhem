package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Cameras;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class SampleFinder implements VisionProcessor, CameraStreamSource {

    public static Scalar yellowLowerBound = new Scalar(10, 100, 100);
    public static Scalar yellowUpperBound = new Scalar(30,255,255);


    public static Scalar redLowerBound = new Scalar(170,100,50);
    public static Scalar redUpperBound = new Scalar(10,255,255);


    public static Scalar blueLowerBound = new Scalar(100,90,90);
    public static Scalar blueUpperBound = new Scalar(160,255,255);

    public static double areaThreshold = 5000;
    public static double centerLine = 320;

    AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

    /** your alliance color (red/blue)*/
    public Alliance alliance;
    /** whether to filter  */
    public boolean filterYellow = true;

    /**The distance (in pixels) of the nearest sample's center; negative means left, positive means right */
    public double nearestSampleDistance = 0;
    /** If the contour is inside the center line */
    public boolean insideContour = false;

    FtcDashboard dashboard;

    enum Alliance {
        red,
        blue
    }
    public SampleFinder(Alliance alliance) {
        this.alliance = alliance;
        this.dashboard = FtcDashboard.getInstance();
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // copy it to avoid affecting future processors
        Mat mat = new Mat();
        frame.copyTo(mat);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        // convert to hsv
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        // filter for (red/blue) or yellow
        Mat filteredMat = new Mat();
        if(alliance == Alliance.red) {
            // Filter from x to 180, or 0 to x because red wraps around
            Mat temp = new Mat();
            Core.inRange(mat, redLowerBound, new Scalar(180, redUpperBound.val[1], redUpperBound.val[2]), temp);
            Core.inRange(mat, new Scalar(0, redLowerBound.val[1], redLowerBound.val[2]), redUpperBound, filteredMat);
            Core.add(filteredMat, temp, filteredMat);
            temp.release();
        } else { // Filter for blue
            Core.inRange(mat, blueLowerBound, blueUpperBound, filteredMat);
        }
        if(filterYellow) { // filter for yellow if set then logical OR with ^
            Mat yellowMat = new Mat();
            Core.inRange(mat, yellowLowerBound, yellowUpperBound, yellowMat);
            Core.add(filteredMat, yellowMat, filteredMat);
            yellowMat.release();
        }
        filteredMat.copyTo(mat);
        filteredMat.release();

        // Get Contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();


        double nearestDistance = Double.POSITIVE_INFINITY;
        insideContour = false;
        for(int i = 0; i < contours.size(); i++ ) {
            // Process contours here

            if(Imgproc.contourArea(contours.get(i)) < areaThreshold) continue;
            Imgproc.drawContours(frame, contours, i, new Scalar(255,0,255), 3);
            Moments moments = Imgproc.moments(contours.get(i));

            // Get centroids of such contours and compute nearest distance of such contours
            double cX = moments.m10/moments.m00;
            double cY = moments.m01/moments.m00;
            Imgproc.drawMarker(frame, new Point(cX, cY), new Scalar(0,255,255));
            if(Math.abs(nearestDistance) > Math.abs(cX - centerLine)) nearestDistance = cX - centerLine;

            // Check if center line intersects the contour
            Point[] points = contours.get(i).toArray();
            boolean leftSideIntersects = false;
            boolean rightSideIntersects = false;
            for (Point point : points) {
                leftSideIntersects = leftSideIntersects || point.x - centerLine <= 0;
                rightSideIntersects = rightSideIntersects || point.x - centerLine >= 0;

                insideContour = insideContour || (leftSideIntersects && rightSideIntersects);
                if (insideContour) break;
            }
        }
        Imgproc.drawMarker(frame, new Point(320,240), new Scalar(0,255,255));
        telemetryPacket.put("Nearest Distance", nearestDistance);
        telemetryPacket.put("Inside Contour", insideContour);
        this.nearestSampleDistance = nearestDistance;
        // TODO: Measure appropriate center line

        dashboard.sendTelemetryPacket(telemetryPacket);

        Bitmap bitmap = Bitmap.createBitmap(frame.width(),  frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, bitmap);
        lastFrame.set(bitmap);
        mat.release();
        return null;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
