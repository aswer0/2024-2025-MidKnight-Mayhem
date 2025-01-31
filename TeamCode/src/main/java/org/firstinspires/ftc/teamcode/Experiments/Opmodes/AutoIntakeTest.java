package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.firstinspires.ftc.teamcode.FinalPrograms.SpecimenAuto;
import org.opencv.core.Point;

@Autonomous
public class AutoIntakeTest extends OpMode {
    public static double sample_x = 24.5;
    public static double sample_y = 37;

    public static double intake_sample_x = 24;
    public static double intake_sample_y = 35;

    public static double pos = 650;
    public static double dist_thresh = 2.5;
    public static double intake_dist_thresh = 4.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 22;

    public static double power = 1;
    public static double gvf_speed = 0.1;
    public static double gvf_thresh = 13;

    public static double target_x = 50; //36.2
    public static double target_y = 90;
    double deposit_state = 0;

    Point target;
    Point get_specimen_target;
    Point get_sample_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    State state = State.intakeSample;
    Lift lift;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Arm arm;

    Alliance alliance = Alliance.red;

    // pid -> deposit -> goToSpecimen / intakeSample (pick up from lines)
    // goToSpecimen -> pickupSpecimen -> pid
    // intakeSample -> goToSpecimen (if done w/ samples) / setupSpitSample
    // setupSpitSample -> spitSample

    enum State {
        intakeSample,
        spitSample,
        setupSpitSample,
        stop
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(11,28),
                new Point(10.3, 65.5),
                new Point(19.2, 68.2),
                new Point(36.5, 75)
        };

        target = new Point(target_x, target_y);
        get_specimen_target = new Point(12.875, 28);
        get_sample_target = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(follow_path, wheelControl, odometry, telemetry, gvf_speed, gvf_thresh, 180, power);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
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
    public void start() {
        timer.reset();
    }

    @Override
    public void loop(){
        switch (state){
            case intakeSample:
                intake.down();
                intake.intake();

                if (timer.milliseconds()>1000) {
                    horizontalSlides.setPosition(horizontal_pos);
                }

                if (intake.hasCorrectSample(false) || timer.milliseconds() >= 2500){ //original 2500
                    timer.reset();
                    state = State.setupSpitSample;
                }
                break;

            case setupSpitSample:
                arm.toIdlePosition();
                intake.reverseDown();

                horizontalSlides.setPosition(0);

                if (timer.milliseconds() > 1000){
                    timer.reset();
                    state = State.spitSample;
                }
                break;

            case spitSample:
                horizontalSlides.setPosition(-470);
                intake.reverseDown();
                intake.reverse();

                if (timer.milliseconds() >= 1000){
                    horizontalSlides.setPosition(-100);
                    timer.reset();
                    state = State.stop;
                }

                break;

            case stop:
                break;
        }

    }
}
