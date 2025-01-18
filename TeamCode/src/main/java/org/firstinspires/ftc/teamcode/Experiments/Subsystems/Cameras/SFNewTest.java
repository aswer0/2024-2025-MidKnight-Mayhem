package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Cameras;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp
@Config
public class SFNewTest extends OpMode {
    SampleFinderNew processor;
    CameraName camera;
    VisionPortal portal;
    FtcDashboard dashboard;

    Odometry odometry;
    WheelControl drive;

    List<LynxModule> allHubs;

    double powerLevel = 1;

    ElapsedTime timer;

    // For drive
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public static boolean filterYellow = true;

    @Override
    public void init() {
        processor = new SampleFinderNew(Alliance.red, 5);
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

        //================================================

        odometry = new Odometry(hardwareMap, 0, 0, 0,"OTOS");
        drive = new WheelControl(hardwareMap, odometry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        timer = new ElapsedTime();
    }
    @Override
    public void loop() {
        odometry.opt.update();
        processor.filterYellow = filterYellow;

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) powerLevel -= 0.1;
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) powerLevel += 0.1;

        drive.drive(gamepad1.left_stick_y, 1.1*gamepad1.left_stick_x, -gamepad1.right_stick_x, 0, powerLevel);
    }


}