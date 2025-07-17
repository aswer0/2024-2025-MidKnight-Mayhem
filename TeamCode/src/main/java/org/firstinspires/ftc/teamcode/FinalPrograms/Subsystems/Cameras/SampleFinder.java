package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Cameras;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
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
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class SampleFinder implements VisionProcessor, CameraStreamSource {

    public static Scalar yellowLowerBound = new Scalar(10, 100, 100);
    public static Scalar yellowUpperBound = new Scalar(30,255,255);

    public static Scalar redLowerBound = new Scalar(170,100,50);
    public static Scalar redUpperBound = new Scalar(10,255,255);


    public static Scalar blueLowerBound = new Scalar(100,90,90);
    public static Scalar blueUpperBound = new Scalar(160,255,255);

    public static double areaThreshold = 3750;
    public static double centerLine = 320;

    public static double depth_scalar = 950;
    public static double horizontal_scalar = 27.75;
    public static double horizontal_offset = 5.5;

    AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

    /** your alliance color (red/blue)*/
    public Alliance alliance;
    /** whether to filter  */
    public boolean filterYellow = true;

    /** Inches to encoder ticks for slides are required*/
    /**The distance (tuned to inches now) of the nearest sample's center; negative means left, positive means right */
    public double nearestSampleDistance = 0;
    /**The depth (tuned to inches now) of the neartest sample horizontally */
    public double nearestSampleDepth = 0;
    /** If the contour is inside the center line */
    public boolean insideContour = false;

    public ArrayList<Contour> contours;

    public class Contour {
        public boolean colored;
        public double size;

    }

    FtcDashboard dashboard;
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
        Mat coloredMat = new Mat();
        frame.copyTo(coloredMat);
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        // convert to hsv
        Imgproc.cvtColor(coloredMat, coloredMat, Imgproc.COLOR_RGB2HSV);
        // filter for (red/blue) or yellow
        Mat filteredMat = new Mat();
        if (alliance == Alliance.red) {
            // Filter from x to 180, or 0 to x because red wraps around
            Mat temp = new Mat();
            Core.inRange(coloredMat, redLowerBound, new Scalar(180, redUpperBound.val[1], redUpperBound.val[2]), temp);
            Core.inRange(coloredMat, new Scalar(0, redLowerBound.val[1], redLowerBound.val[2]), redUpperBound, filteredMat);
            Core.add(filteredMat, temp, filteredMat);
            temp.release();
        } else { // Filter for blue
            Core.inRange(coloredMat, blueLowerBound, blueUpperBound, filteredMat);
        }
        Mat yellowMat = new Mat();
        Core.inRange(coloredMat, yellowLowerBound, yellowUpperBound, yellowMat);
        filteredMat.copyTo(coloredMat);
        filteredMat.release();

        // Get Contours
        ArrayList<MatOfPoint> coloredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(coloredMat, coloredContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        hierarchy = new Mat();
        ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
        Imgproc.findContours(yellowMat, yellowContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();


        double nearestDistance = Double.POSITIVE_INFINITY;
        double nearestSampleDepth = Double.POSITIVE_INFINITY;
        insideContour = false;
        ArrayList<Contour> processedContours = new ArrayList<>();
        for(int i = 0; i < coloredContours.size(); i++ ) {
            // Process contours here
            double area = Imgproc.contourArea(coloredContours.get(i));

            if(area < areaThreshold) continue;
            Moments moments = Imgproc.moments(coloredContours.get(i));

            // Get centroids of such contours and compute nearest distance of such contours
            double cX = moments.m10/moments.m00;
            double cY = moments.m01/moments.m00;
            // split the vision in half so we don't cross to the other alliance's thing
            if (cX - centerLine < 0) continue;
            Imgproc.drawMarker(frame, new Point(cX, cY), new Scalar(0,255,255));
            Imgproc.drawContours(frame, coloredContours, i, new Scalar(255,0,255), 3);
            if(Math.abs(nearestDistance) > Math.abs(cX - centerLine)){
                nearestSampleDepth = depth_scalar / /*(1 - b_scalar*cY)*/ Math.sqrt(area);
                nearestDistance = (cX - centerLine) / horizontal_scalar - horizontal_offset;
            }

            // Check if center line intersects the contour
            Point[] points = coloredContours.get(i).toArray();
            boolean leftSideIntersects = false;
            boolean rightSideIntersects = false;
            for (Point point : points) {
                leftSideIntersects = leftSideIntersects || point.x - centerLine <= 0;
                rightSideIntersects = rightSideIntersects || point.x - centerLine >= 0;

                insideContour = insideContour || (leftSideIntersects && rightSideIntersects);
                if (insideContour) break;
            }
            Contour contour = new Contour();
            contour.colored = true;
            contour.size = Imgproc.contourArea(coloredContours.get(i));
            processedContours.add(contour);
        }
        for(int i = 0; i < yellowContours.size(); i++ ) {
            // Process contours here
            double area = Imgproc.contourArea(yellowContours.get(i));

            if(area < areaThreshold) continue;
            Moments moments = Imgproc.moments(yellowContours.get(i));

            // Get centroids of such contours and compute nearest distance of such contours
            double cX = moments.m10/moments.m00;
            double cY = moments.m01/moments.m00;
            // split the vision in half so we don't cross to the other alliance's thing
            if (cX - centerLine < 0) continue;
            Imgproc.drawMarker(frame, new Point(cX, cY), new Scalar(0,255,255));
            Imgproc.drawContours(frame, yellowContours, i, new Scalar(255,0,255), 3);
            if(Math.abs(nearestDistance) > Math.abs(cX - centerLine)){
                nearestSampleDepth = depth_scalar / Math.sqrt(area);
                nearestDistance = (cX - centerLine) / horizontal_scalar - horizontal_offset;
            }
            Contour contour = new Contour();
            contour.colored = true;
            contour.size = Imgproc.contourArea(yellowContours.get(i));
            processedContours.add(contour);
            if(!filterYellow) continue;
            // Check if center line intersects the contour
            Point[] points = yellowContours.get(i).toArray();
            boolean leftSideIntersects = false;
            boolean rightSideIntersects = false;
            for (Point point : points) {
                leftSideIntersects = leftSideIntersects || point.x - centerLine <= 0;
                rightSideIntersects = rightSideIntersects || point.x - centerLine >= 0;

                insideContour = insideContour || (leftSideIntersects && rightSideIntersects);
                if (insideContour) break;
            }
        }
        this.contours = processedContours;
        Imgproc.drawMarker(frame, new Point(320,240), new Scalar(0,255,255));
        telemetryPacket.put("Nearest Distance", nearestDistance);
        telemetryPacket.put("Depth", nearestSampleDepth);
        telemetryPacket.put("Inside Contour", insideContour);
        telemetryPacket.put("Num Contours", contours.size());
        this.nearestSampleDistance = nearestDistance;
        this.nearestSampleDepth = nearestSampleDepth;

        dashboard.sendTelemetryPacket(telemetryPacket);

        Bitmap bitmap = Bitmap.createBitmap(frame.width(),  frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, bitmap);
        lastFrame.set(bitmap);
        coloredMat.release();
        yellowMat.release();
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
