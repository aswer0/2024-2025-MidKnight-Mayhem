package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class SampleAuto extends OpMode {
    Point start_point;
    //Point get_specimen_target;
    Point deposit_sample_target;
    Point hang_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    BCPath bcpath;
    VectorField vf;

    State state = State.pid;
    Lift lift;
    Manipulator manipulator;

    Intake intake;
    //Arm arm;

    enum State {
        pid,
        deposit_sample,
        intake_sample,
        hang
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void init() {
        start_point = new Point(7.875, 96);
        deposit_sample_target = new Point(12+5, 132-5);
        hang_target = new Point(72, 96);

        Point[][] follow_path = {{
                new Point(7.875, 21.3),
                new Point(48, 21.3),
                new Point(3.7, 28.6),
                new Point(43.6, 47.3),
                new Point(16.5, 72.4),
                new Point(23, 65),
        }};

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, start_point.x, start_point.y, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        bcpath = new BCPath(follow_path);
        vf = new VectorField(wheelControl, odometry, bcpath, 135);

        //path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, -180, 0.7);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);

        intake = new Intake(hardwareMap, sensors);
        //arm = new Arm(hardwareMap);

        manipulator.closeClaw();
    }

    @Override
    public void loop() {
        odometry.opt.update();
        switch(state) {
            case pid:
                intake.up();
                intake.stop();
                manipulator.closeClaw();
                //lift.setPosition(1600);

                vf.move_to_point(deposit_sample_target, 135, 0.5);

                if (vf.dist_to_end() < 1 || timer.milliseconds() > 5000) {
                    timer.reset();
                    state = State.deposit_sample;
                }

            case deposit_sample:
                if (timer.milliseconds() > 100) {
                    manipulator.openClaw();
                }

                if (timer.milliseconds() > 1000) {
                    timer.reset();
                    state = State.hang;
                }

            case hang:
                manipulator.closeClaw();
                lift.setPosition(1200);
                vf.move_to_point(hang_target, -90, 0.5);
        }
        lift.update();
        intake.up();
        telemetry.update();
    }
}
