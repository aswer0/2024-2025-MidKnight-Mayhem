package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Cameras;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SampleFinderV2 implements VisionProcessor {
    // Set up telemetry
    public Telemetry telemetry;

    // Color thresholds
    public Scalar YELLOW_LOWER = new Scalar(16, 157, 220);
    public Scalar YELLOW_UPPER = new Scalar(48, 255, 255);
    public Scalar RED_LOWER = new Scalar(175, 110, 181);
    public Scalar RED_UPPER = new Scalar(5, 255, 255);
    public Scalar BLUE_LOWER = new Scalar(109, 89, 45);
    public Scalar BLUE_UPPER = new Scalar(133, 255, 255);

    // Canny thresholds
    public double LOW_CANNY = 70;
    public double HIGH_CANNY = 150;

    // Saved Mats
    public Mat src_HSV = new Mat();

    // Alliance
    public enum Alliance {red, blue};
    public Alliance alliance = Alliance.red;

    // Bounding rectangles
    public ArrayList<RotatedRect> yellow_rects = new ArrayList<>();
    public ArrayList<RotatedRect> alliance_rects = new ArrayList<>();

    // Minimum bounding box area
    public double MIN_AREA_RATIO = 0.001;
    public double min_area;

    // Modifies inRange to take hue into account as it's circular
    public void inRangeNew(Mat src, Scalar lowerb, Scalar upperb, Mat dst) {
        if (lowerb.val[0] > upperb.val[0]) {
            Mat range1 = new Mat();
            Mat range2 = new Mat();
            Core.inRange(src, lowerb, new Scalar(360, upperb.val[1], upperb.val[2]), range1);
            Core.inRange(src, new Scalar(0, lowerb.val[1], lowerb.val[2]), upperb, range2);
            Core.add(range1, range2, dst);
            range1.release();
            range2.release();
        } else {
            Core.inRange(src, lowerb, upperb, dst);
        }
    }

    public void drawRotatedRect(Canvas canvas, float scaleBmpPxToCanvasPx, RotatedRect rect, Paint paint) {
        Point[] vertices = new Point[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            float x1 = (float)vertices[i].x*scaleBmpPxToCanvasPx;
            float y1 = (float)vertices[i].y*scaleBmpPxToCanvasPx;
            float x2 = (float)vertices[(i+1)%4].x*scaleBmpPxToCanvasPx;
            float y2 = (float)vertices[(i+1)%4].y*scaleBmpPxToCanvasPx;
            canvas.drawLine(x1, y1, x2, y2, paint);
        }
    }

    public SampleFinderV2(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Gaussian blur image
        Mat src = new Mat();
        //frame.copyTo(src);
        Imgproc.blur(frame, src, new Size(3, 3));

        // Get min contour area
        min_area = src.rows()*src.cols()*MIN_AREA_RATIO;

        // Convert to HSV for easier filtering
        Imgproc.cvtColor(src, src_HSV, Imgproc.COLOR_RGB2HSV);

        // Mask with filters
        Mat yellow_filter = new Mat();
        Mat alliance_filter = new Mat();
        inRangeNew(src_HSV, YELLOW_LOWER, YELLOW_UPPER, yellow_filter);
        if (alliance == Alliance.red) {
            inRangeNew(src_HSV, RED_LOWER, RED_UPPER, alliance_filter);
        } else {
            inRangeNew(src_HSV, BLUE_LOWER, BLUE_UPPER, alliance_filter);
        }

        Mat yellow_masked = new Mat();
        Mat alliance_masked = new Mat();
        Core.bitwise_and(src, src, yellow_masked, yellow_filter);
        Core.bitwise_and(src, src, alliance_masked, alliance_filter);
        yellow_filter.release();
        alliance_filter.release();

        // Use Canny edge detection
        Mat yellow_canny = new Mat();
        Mat alliance_canny = new Mat();
        Imgproc.Canny(yellow_masked, yellow_canny, LOW_CANNY, HIGH_CANNY);
        Imgproc.Canny(alliance_masked, alliance_canny, LOW_CANNY, HIGH_CANNY);
        //yellow_canny.copyTo(frame);
        yellow_masked.release();
        alliance_masked.release();

        // Get contours
        Mat hierarchy = new Mat();
        ArrayList<MatOfPoint> yellow_contours = new ArrayList<MatOfPoint>();
        ArrayList<MatOfPoint> alliance_contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(yellow_canny, yellow_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(alliance_canny, alliance_contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        yellow_canny.release();
        alliance_canny.release();
        hierarchy.release();

        //yellow_masked.copyTo(frame);

        // Get contours directly from masked picture
        /*Mat hierarchy = new Mat();
        ArrayList<MatOfPoint> yellow_contours = new ArrayList<MatOfPoint>();
        ArrayList<MatOfPoint> alliance_contours = new ArrayList<MatOfPoint>();
        Imgproc.cvtColor(yellow_masked, yellow_masked, Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(alliance_masked, alliance_masked, Imgproc.COLOR_RGB2GRAY);
        Imgproc.findContours(yellow_masked, yellow_contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(alliance_masked, alliance_contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yellow_masked.release();
        alliance_masked.release();
        hierarchy.release();*/

        /*Mat hierarchy = new Mat();
        ArrayList<MatOfPoint> yellow_contours = new ArrayList<MatOfPoint>();
        ArrayList<MatOfPoint> alliance_contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(yellow_filter, yellow_contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(alliance_filter, alliance_contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yellow_filter.release();
        alliance_filter.release();
        hierarchy.release();*/


        // Get bounding rectangles
        yellow_rects.clear();
        alliance_rects.clear();
        for (int i = 0; i < yellow_contours.size(); i++) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(yellow_contours.get(i).toArray()));
            if (rect.size.width*rect.size.height < min_area) continue;
            yellow_rects.add(rect);
        }
        for (int i = 0; i < alliance_contours.size(); i++) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(alliance_contours.get(i).toArray()));
            if (rect.size.width*rect.size.height < min_area) continue;
            alliance_rects.add(rect);
        }

        // No return value needed
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {
        // Yellow draw rects
        Paint yellow_paint = new Paint();
        yellow_paint.setColor(Color.YELLOW);
        yellow_paint.setStyle(Paint.Style.STROKE);
        yellow_paint.setStrokeWidth(scaleCanvasDensity * 4);
        for (RotatedRect rect : yellow_rects) {
            drawRotatedRect(canvas, scaleBmpPxToCanvasPx, rect, yellow_paint);
        }

        // Alliance draw rects
        Paint alliance_paint = new Paint();
        if (alliance == Alliance.red) {
            alliance_paint.setColor(Color.RED);
        } else {
            alliance_paint.setColor(Color.BLUE);
        }
        alliance_paint.setStyle(Paint.Style.STROKE);
        alliance_paint.setStrokeWidth(scaleCanvasDensity * 4);
        for (RotatedRect rect : alliance_rects) {
            drawRotatedRect(canvas, scaleBmpPxToCanvasPx, rect, alliance_paint);
        }
    }
}
