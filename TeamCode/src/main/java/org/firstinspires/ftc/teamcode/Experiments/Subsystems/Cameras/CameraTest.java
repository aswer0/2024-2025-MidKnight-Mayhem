package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Cameras;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class CameraTest extends OpMode {
    SampleFinder processor;
    CameraName camera;
    VisionPortal portal;
    FtcDashboard dashboard;

    @Override
    public void init() {
        processor = new SampleFinder(SampleFinder.Alliance.red);
        camera = hardwareMap.get(CameraName.class, "Webcam 1");
        portal = new VisionPortal
                .Builder()
                .addProcessor(processor)
                .setCamera(camera)
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(processor, 0);
    }
    @Override
    public void loop() {
    }


}