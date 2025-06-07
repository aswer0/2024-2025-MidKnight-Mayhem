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

@TeleOp
@Config
public class CameraTest extends OpMode {
    SampleFinder processor;
    CameraName camera;
    VisionPortal portal;
    FtcDashboard dashboard;

    Odometry odometry;
    WheelControl drive;

    List<LynxModule> allHubs;

    double powerLevel = 1;
    Point target_pos;

    ElapsedTime timer;

    Sensors sensors;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Path path;
    Point[] cp = {
            new Point(0, 0)
    };

    public static double p = 1.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double inch_to_ticks = 35;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public static boolean filterYellow = true;

    @Override
    public void init() {
        processor = new SampleFinder(Alliance.red);
        camera = hardwareMap.get(CameraName.class, "Webcam 1");
        portal = new VisionPortal
                .Builder()
                .addProcessor(processor)
                .setCamera(camera)
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(processor, 60);

        odometry = new Odometry(hardwareMap, 0, 0, 0,"OTOS");
        drive = new WheelControl(hardwareMap, odometry);

        horizontalSlides = new HorizontalSlides(hardwareMap);
        sensors = new Sensors(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, sensors);
        path = new Path(cp, drive, odometry, telemetry, 0, 0, 0, powerLevel);
        intake.setAlliance(Alliance.red);

        allHubs = hardwareMap.getAll(LynxModule.class);

        timer = new ElapsedTime();
    }
    public double get_dist(Point p1, Point p2){
        return Math.sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
    }
    public boolean at_point(double hor, double collision_dist){
        return get_dist(
                new Point(odometry.opt.get_x(), odometry.opt.get_y()),
                new Point(odometry.opt.get_x(), hor)
        ) <= collision_dist;
    }

    @Override
    public void loop() {
        odometry.opt.update();
        processor.filterYellow = filterYellow;

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper) {
            path.pid_to_point(target_pos, 0);

            if (path.at_point(target_pos, 0.5)) {
                if (!intake.hasCorrectSample(true)) {
                    horizontalSlides.setPosition(-processor.nearestSampleDepth * inch_to_ticks);
                    intake.down();
                    intake.intake();
                } else {
                    intake.up();
                    horizontalSlides.setPosition(0);
                }
            }
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
            target_pos = new Point(odometry.opt.get_x(), odometry.opt.get_y()+processor.nearestSampleDistance);
        }

        drive.drive(gamepad1.left_stick_y, 1.1 * gamepad1.left_stick_x, -gamepad1.right_stick_x, 0, powerLevel);

        telemetry.addData("Nearest Distance", processor.nearestSampleDistance);
        telemetry.addData("Nearest Distance Depth", processor.nearestSampleDepth);
        telemetry.addData("Target Position", target_pos);
        telemetry.addData("Target Depth Position", -processor.nearestSampleDepth * inch_to_ticks);
    }
}