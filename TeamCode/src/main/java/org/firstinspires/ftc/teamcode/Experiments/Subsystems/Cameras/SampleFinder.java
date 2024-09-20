package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Cameras;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

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

    public static Scalar yellowLowerBound = new Scalar(0,0,0);
    public static Scalar yellowUpperbound = new Scalar(0,0,0);


    public static Scalar redLowerBound = new Scalar(0,0,0);
    public static Scalar redUpperBound = new Scalar(0,0,0);


    public static Scalar blueLowerBound = new Scalar(0,0,0);
    public static Scalar blueUpperBound = new Scalar(0,0,0);
    AtomicReference<Bitmap> lastFrame = new AtomicReference<>();
    public Alliance alliance;
    /** whether to filter  */
    public boolean filterYellow = false;
    /** The direction of the nearest sample*/
    public Direction nearestSampleDirection = Direction.left;
    /**The distance (in pixels) of the nearest sample's center */
    public double nearestSampleDistance = 0;

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
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // copy it to avoid affecting future processors
        Mat mat = new Mat();
        frame.copyTo(mat);
        // filter color
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        //


        Bitmap bitmap = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, bitmap);
        lastFrame.set(bitmap);
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
