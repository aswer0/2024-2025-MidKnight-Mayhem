package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Cameras;


import android.graphics.Point;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.Intake;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision {
    WheelControl wheelControl;
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

    double target_position;
    double inch_to_ticks = 35;
    int sign;

    enum State{
        setPosition,
        moveHorizontal,
        intake,
        retract
    }

    public Vision(Odometry odometry, Telemetry telemetry, WheelControl wheelControl, HardwareMap hardwareMap, HorizontalSlides horizontalSlides, Intake intake){
        this.telemetry = telemetry;
        this.wheelControl = wheelControl;
        this.horizontalSlides = horizontalSlides;
        this.intake = intake;
        this.odometry = odometry;

        this.processor = new SampleFinder(intake.alliance);
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
    }

    public boolean get_sample(double powerLevel, double collision_threshold, char orientation){
        switch (state){
            case setPosition:
                target_position = odometry.opt.get_y()+processor.nearestSampleDistance;
                sign = (int)(processor.nearestSampleDistance/Math.abs(processor.nearestSampleDistance));

                state = State.moveHorizontal;

                break;

            case moveHorizontal:
                wheelControl.drive(0, sign, 0, 0, powerLevel);

                if (Math.abs(odometry.opt.get_x()-target_position) <= collision_threshold && orientation == 'x'){
                    state = State.intake;
                    timer.reset();
                }
                else{
                    if (Math.abs(odometry.opt.get_y()-target_position) <= collision_threshold){
                        state = State.intake;
                        timer.reset();
                    }
                }

                break;

            case intake:
                horizontalSlides.setPosition(-processor.nearestSampleDepth * inch_to_ticks);
                if (timer.milliseconds() >= 350){
                    intake.down();
                    intake.intake();

                    if (intake.hasCorrectSample(true)) {
                        state = State.retract;
                        timer.reset();
                    }
                }
                break;

            case retract:
                intake.up();
                horizontalSlides.setPosition(0);

                if (timer.milliseconds() >= 350){
                    return true;
                }
                break;

        };

        return false;
    }

}
