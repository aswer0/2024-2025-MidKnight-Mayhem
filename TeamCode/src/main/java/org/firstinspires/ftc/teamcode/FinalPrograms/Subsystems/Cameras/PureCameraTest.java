package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Cameras;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.List;
import java.util.Optional;

@TeleOp
@Config
public class PureCameraTest extends OpMode {
    SampleFinder processor;
    CameraName camera;
    VisionPortal portal;
    FtcDashboard dashboard;
    HorizontalSlides slides;
    boolean lastExtendSlides = false;

    List<LynxModule> allHubs;


    public static boolean filterYellow = false;
    public static boolean extendSlides = false;

    @Override
    public void init() {
        processor = new SampleFinder(Alliance.blue);
        camera = hardwareMap.get(CameraName.class, "Webcam 1");
        slides = new HorizontalSlides(hardwareMap);
        portal = new VisionPortal
                .Builder()
                .addProcessor(processor)
                .setCamera(camera)
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(processor, 60);
        slides = new HorizontalSlides(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
    }

    @Override
    public void loop() {
        processor.filterYellow = filterYellow;

        telemetry.addData("Nearest Distance", processor.nearestSampleDistance);
        telemetry.addData("slides range", slides.horizontalSlidesMotor.getCurrentPosition());
        if(extendSlides != lastExtendSlides) slides.setPosition(processor.nearestSampleDepth);
        lastExtendSlides = extendSlides;
    }
}