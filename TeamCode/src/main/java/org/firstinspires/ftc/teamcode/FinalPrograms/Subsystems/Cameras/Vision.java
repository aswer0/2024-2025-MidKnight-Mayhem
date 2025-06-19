package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Cameras;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.Optional;

public class Vision {
    VectorField vf;
    Telemetry telemetry;
    Odometry odometry;
    HorizontalSlides horizontalSlides;
    Intake intake;

    SampleFinder processor;
    CameraName camera;
    VisionPortal portal;
    FtcDashboard dashboard;

    State state = State.setPosition;
    ElapsedTime timer;

    Point target_position;
    double target_depth;
    double inch_to_ticks = 35;

    enum State{
        setPosition,
        move,
        intake,
        retract
    }

    public Vision(Odometry odometry, Telemetry telemetry, VectorField vectorField, HardwareMap hardwareMap, HorizontalSlides horizontalSlides, Intake intake, Alliance a){
        this.telemetry = telemetry;
        this.vf = vectorField;
        this.horizontalSlides = horizontalSlides;
        this.intake = intake;
        this.odometry = odometry;

        this.processor = new SampleFinder(a);
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

        timer = new ElapsedTime();
    }

    public boolean get_sample(double powerLevel, double target_angle, double collision_threshold, char orientation){
        switch (state){
            case setPosition:
                if (orientation == 'x'){
                    target_position = new Point(
                            odometry.opt.get_x()+processor.nearestSampleDistance,
                            odometry.opt.get_y()
                    );
                }
                else{
                    target_position = new Point(
                            odometry.opt.get_x(),
                            odometry.opt.get_y()+processor.nearestSampleDistance
                    );
                }
                target_depth = -processor.nearestSampleDepth*inch_to_ticks;

                state = State.move;

                break;

            case move:
                intake.up();
                vf.pid_to_point(target_position, target_angle, powerLevel);

                if (vf.at_point(target_position, collision_threshold)){
                    timer.reset();
                    state = State.intake;
                }

                break;

            case intake:
                vf.pid_to_point(target_position, target_angle, powerLevel);
                horizontalSlides.setPosition(target_depth);

                if (timer.milliseconds() >= 1000){
                    intake.down();
                    intake.intake();

                    if (intake.hasCorrectSample(true)) {
                        timer.reset();
                        state = State.retract;
                    }
                }
                else{
                    intake.up();
                }

                break;

            case retract:
                intake.up();

                if (timer.milliseconds() >= 500){
                    horizontalSlides.setPosition(0);
                }
                if (timer.milliseconds() >= 800 && timer.milliseconds() <= 860){
                    intake.reverse();
                }
                else{
                    intake.intake();
                }

                if (timer.milliseconds() >= 1000){
                    intake.stop();
                    return true;
                }
                break;

        };

        return false;
    }

    public SampleFinder get_processor(){
        return this.processor;
    }
    public void filter_yellow(boolean filterYellow){
        this.processor.filterYellow = filterYellow;
    }
    public void reset(){
        state = State.setPosition;
    }

}
