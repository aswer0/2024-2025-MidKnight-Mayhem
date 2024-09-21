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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class SampleFinder implements VisionProcessor, CameraStreamSource {

    public static Scalar yellowLowerBound = new Scalar(10, 100, 100);
    public static Scalar yellowUpperBound = new Scalar(30,255,255);


    public static Scalar redLowerBound = new Scalar(170,100,50);// possibly split this
    public static Scalar redUpperBound = new Scalar(10,255,255);


    public static Scalar blueLowerBound = new Scalar(100,90,90);
    public static Scalar blueUpperBound = new Scalar(160,255,255);

    AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

    public Alliance alliance;
    /** whether to filter  */
    public boolean filterYellow = true;
    /** The direction of the nearest sample*/

    public Direction nearestSampleDirection = Direction.left;
    /**The distance (in pixels) of the nearest sample's center */
    public double nearestSampleDistance = 0;

    FtcDashboard dashboard;

    enum Alliance {
        red,
        blue
    }
    enum Direction {
        left,
        right
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

        // convert to hsv
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        // filter for (red/blue) or yellow
        Mat allianceMat = new Mat();
        if(alliance == Alliance.red) {
            // Filter from x to 180, or 0 to x because red wraps aroound
            Mat temp = new Mat();
            Core.inRange(mat, redLowerBound, new Scalar(180, redUpperBound.val[1], redUpperBound.val[2]), temp);
            Core.inRange(mat, new Scalar(0, redLowerBound.val[1], redLowerBound.val[2]), redUpperBound, allianceMat);
            Core.add(allianceMat, temp, allianceMat);
            temp.release();
        } else { // Filter for blue
            Core.inRange(mat, blueLowerBound, blueUpperBound, allianceMat);
        }
        if(filterYellow) { // filter for yellow if set then logical OR with ^
            Mat yellowMat = new Mat();
            Core.inRange(mat, yellowLowerBound, yellowUpperBound, yellowMat);
            Core.add(allianceMat, yellowMat, allianceMat);
            yellowMat.release();
        }
        allianceMat.copyTo(mat);
        allianceMat.release();

        // TODO: start contouring and return LEFT or RIGHT and distance.
        // TODO: Measure appropriate center line

        Bitmap bitmap = Bitmap.createBitmap(mat.width(),  mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, bitmap);
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
