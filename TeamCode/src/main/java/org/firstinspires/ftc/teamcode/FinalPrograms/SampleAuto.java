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
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
//import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class SampleAuto extends OpMode {
    double intake_slide_extend = -450;
    double corner_offset = 18;
    Point start_point;
    double get_sample_x, get_sample_y;
    Point deposit_sample_target;
    Point park_target;
//
    int deposit_state;
    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    BCPath bcpath;
    VectorField vf;

    State state = State.pid;
    Lift lift;
    Arm arm;
    HorizontalSlides intakeSlides;

    Intake intake;

    Alliance alliance = Alliance.red;

    enum State {
        pid,
        deposit_sample,
        move_to_intake_sample,
        intake_sample,
        transfer_sample,
        park
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void init() {
        start_point = new Point(7.875, 96);
        get_sample_x = 25;
        get_sample_y = 120;
        deposit_state = 0;
        deposit_sample_target = new Point(corner_offset, 144-corner_offset);
        park_target = new Point(20, 40);
        //ascent_target = new Point(72, 96);

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
        vf = new VectorField(wheelControl, odometry, bcpath, 135, false);

        //path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, -180, 0.7);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        intakeSlides = new HorizontalSlides(hardwareMap);

        arm.toAutoStartPosition();
        arm.closeClaw();
    }

    @Override
    public void init_loop() {
        if (gamepad1.left_bumper) {
            alliance = Alliance.blue;
        } else if (gamepad1.right_bumper) {
            alliance = Alliance.red;
        }

        telemetry.addData("Alliance", alliance);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        lift.update();
        telemetry.update();

        switch(state) {
            case pid:
                intake.up();
                intake.stop();
                arm.closeClaw();
                arm.outtakeSample();
                lift.toHighBasket();

                vf.move_to_point(deposit_sample_target, 135, 0.7);

                if (vf.dist_to_end() < 1 || timer.milliseconds() > 5000) {
                    timer.reset();
                    state = State.deposit_sample;
                }

                break;

            case deposit_sample:
                intake.up();
                intake.stop();
                arm.openClaw();
                arm.outtakeSample();

                if (timer.milliseconds() > 300) {
                    timer.reset();
                    deposit_state++;
                    if (deposit_state <= 3) {
                        state = State.move_to_intake_sample;
                    } else {
                        state = State.park;
                    }
                }

                break;

            case move_to_intake_sample:
                intake.up();
                intake.stop();

                vf.move_to_point(new Point(get_sample_x, get_sample_y), 0, 0.7);

                if (vf.dist_to_end() < 1 || timer.milliseconds() > 5000) {
                    timer.reset();
                    state = State.intake_sample;
                }

                break;

            case intake_sample:
                arm.intakeSample();
                arm.openClaw();
                intake.down();
                intake.intake();
                intakeSlides.setPosition(intake_slide_extend);

                if (intake.hasCorrectSample(true) || timer.milliseconds() >= 2000) {
                    timer.reset();
                    get_sample_y += 10;
                    state = State.transfer_sample;
                }

                break;

            case transfer_sample:
                intakeSlides.setPosition(0);
                intake.up();
                arm.intakeSample();

                if (timer.milliseconds() >= 1000) {
                    arm.closeClaw();
                }
                if (timer.milliseconds() >= 1300) {
                    timer.reset();
                    state = State.pid;
                }

                break;

            case park:
                vf.move_to_point(park_target, 0, 0.7);
        }
    }
}
