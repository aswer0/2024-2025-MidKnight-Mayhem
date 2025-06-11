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
public class CameraTest extends OpMode {
    SampleFinder processor;
    CameraName camera;
    VisionPortal portal;
    FtcDashboard dashboard;

    Odometry odometry;
    WheelControl drive;

    List<LynxModule> allHubs;

    public static double hp = 0.0085, hi = 0, hd = 0.00004;
    PIDController heading;

    public static double powerLevel = 0.5;
    public static double drivePower = 1;
    Point target_pos;

    ElapsedTime timer;

    Sensors sensors;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Path path;
    Point[] cp = {
            new Point(0, 0)
    };
    VectorField vf;

    public static double inch_to_ticks = 35;

    double sign;
    boolean up_trigger = false;

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
        heading = new PIDController(hp, hi, hd);

        horizontalSlides = new HorizontalSlides(hardwareMap);
        sensors = new Sensors(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, sensors);
        path = new Path(cp, drive, odometry, telemetry, 0, 0, 0, powerLevel);
        vf = new VectorField(drive, odometry);
        intake.setAlliance(Alliance.red);

        allHubs = hardwareMap.getAll(LynxModule.class);

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        odometry.opt.update();
        horizontalSlides.update();

        processor.filterYellow = filterYellow;

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper) {
            vf.pid_to_point(target_pos, -90, powerLevel);

            if (vf.at_point(target_pos, 0.75)){
                if (!intake.hasCorrectSample(true)) {
                    horizontalSlides.setPosition(-processor.nearestSampleDepth * inch_to_ticks);
                    intake.down();
                    intake.intake();
                    timer.reset();
                } else {
                    up_trigger = true;
                    if (timer.milliseconds() >= 1200 && timer.milliseconds() <= 1260){
                        intake.reverse();
                    }
                    intake.up();
                    if (timer.milliseconds() >= 750){
                        horizontalSlides.setPosition(0);
                    }
                }
            }
            else{
                intake.up();
            }
        }
        else{
            if (!currentGamepad1.right_bumper){
                intake.up();
            }
        }

        if (currentGamepad1.right_bumper && !previousGamepad2.right_bumper){
            timer.reset();
        }
        if (currentGamepad1.right_bumper){
            intake.down();
            if (timer.milliseconds() >= 500){
                target_pos = new Point(odometry.opt.get_x()+processor.nearestSampleDistance, odometry.opt.get_y());
            }
            drive.drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x, Math.toRadians(odometry.opt.get_heading()), powerLevel);
        }

        telemetry.addData("Nearest Distance", processor.nearestSampleDistance);
        telemetry.addData("Nearest Distance Depth", processor.nearestSampleDepth);
        telemetry.addData("Target Position", target_pos);
        telemetry.addData("Target Sign", sign);
        telemetry.addData("Target Depth Position", -processor.nearestSampleDepth * inch_to_ticks);
        telemetry.addData("x", odometry.opt.get_x());
        telemetry.addData("y", odometry.opt.get_y());
        telemetry.addData("h", odometry.opt.get_heading());
        telemetry.addData("has sample", !intake.hasCorrectSample(true));
        telemetry.addData("up trigger", up_trigger);
        telemetry.addData("gamepad 1 x value", 1.2 * gamepad1.left_stick_x);
    }
}