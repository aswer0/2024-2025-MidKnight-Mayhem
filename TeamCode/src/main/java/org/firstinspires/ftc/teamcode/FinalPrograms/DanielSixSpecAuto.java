package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class DanielSixSpecAuto extends OpMode {
    //public static double sample_x = 24.5;
    //public static double sample_y = 37;

    public static Point intake_sample_1 = new Point(26, 37);
    public static Point intake_sample_2 = new Point(26, 27);
    public static Point intake_sample_3 = new Point(26, 17);
    public static double sweep_sample_x = 32;
    public static double sweep_sample_y = 34;

    public static double pos = 650;
    public static double dist_thresh = 2.5;
    public static double intake_dist_thresh = 2.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 136;
    public static double target_angle_spit = 22;

    public static double power = 1;
    
    public static double target_x = 50; //36.2
    public static double target_y = 90;
    double intake_state = 0;

    Point target;
    Point get_specimen_target;
    //Point get_sample_target;

    ElapsedTime timer;
    ElapsedTime intakeShakeTimer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField vf;

    State state = State.pid;
    Lift lift;
    Intake intake;
    HorizontalSlides intake_slides;
    Arm arm;

    Alliance alliance = Alliance.red;

    // pid -> deposit -> goToSpecimen / intakeSample (pick up from lines)
    // goToSpecimen -> pickupSpecimen -> pid
    // intakeSample -> goToSpecimen (if done w/ samples) / setupSpitSample
    // setupSpitSample -> spitSample

    enum State {
        pid,
        goToSpecimen,
        pickupSpecimen,
        deposit,
        subIntake,
        intakeSample,
        spitSample,
        setupSpitSample,
        park
    }

    @Override
    public void init() {
        target = new Point(target_x, target_y);
        get_specimen_target = new Point(14.875, 24);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        intake_slides = new HorizontalSlides(hardwareMap);
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
    public void loop() {
        odometry.opt.update();
        intake_slides.update();
        lift.update();

        // Every case should include lift, intake, intake slides, intake door, arm, drive info
        switch (state) {
            case pid:
                lift.toHighChamber();
                intake.closeDoor();

                if (intake_state == 0) {
                    arm.outtakeSpecimen1();
                    vf.pid_to_point(target, 0, 1);
                    intake_slides.setPosition(-100);
                    if (sensors.get_front_dist() < dist_thresh) {
                        timer.reset();
                        intake_state++;
                        state = State.subIntake;
                    }
                }

                break;

            case subIntake:
                boolean intake_correct = intake.smarterIntake(false);
                intake_slides.setPosition(horizontal_pos);
                if (intake_correct || timer.milliseconds() >= 2000) {
                    intake_slides.setPosition(0);
                    timer.reset();
                    state = State.goToSpecimen;
                }

            case goToSpecimen:
                intake.stop();
                intake.up();
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("Deposit State", intake_state);
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("Horizontal Motor position: ", intake_slides.horizontalSlidesMotor.getCurrentPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Back Distance", sensors.get_back_dist());
        telemetry.addData("timer", timer.milliseconds());

        lift.update();
        telemetry.update();
        intake_slides.update();
    }
}