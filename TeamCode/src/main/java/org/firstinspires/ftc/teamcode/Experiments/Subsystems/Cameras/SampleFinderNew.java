package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Cameras;

import android.util.Size;

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

@Config
public class SampleFinderNew implements VisionProcessor, CameraStreamSource {
    public Alliance alliance;
    FtcDashboard dashboard;

    // Camera properties (fov in rad)
    public double cam_h;
    public double fov_w;
    public double fov_h;
    public double res_w;
    public double res_h;


    public SampleFinderNew(Alliance alliance, double cam_h,
                           double fov_w, double fov_h, double res_w, double res_h) {
        this.alliance = alliance;
        this.dashboard = FtcDashboard.getInstance();
        this.cam_h = cam_h;
        this.fov_w = fov_w;
        this.fov_h = fov_h;
        this.res_w = res_w;
        this.res_h = res_h;
    }

    public Point pix_to_irl(double x, double y) throws Exception {
        if (y >= 0) throw new Exception("y position of pixel cannot be >0 (above horizon)");
        double horiz_focus_dist = this.res_w/(2*Math.tan(fov_w/2));
        double focus_hypot = Math.hypot(horiz_focus_dist, x);
        double sin_horiz = x/focus_hypot;
        double cos_horiz = horiz_focus_dist/focus_hypot;
        double dist_from_cam = Math.abs(cam_h*this.res_h/(2*y*Math.tan(fov_h/2)));
        return new Point(dist_from_cam*sin_horiz, dist_from_cam*cos_horiz);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

    }

    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    @Override
    public Object processFrame(Mat frame) {
        Mat coloredMat = new Mat();
        frame.copyTo(coloredMat);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }
}
