package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Config
@Autonomous
public class FourSampleAuto extends OpMode {
    Point sample_point;
    double intake_angle = 90;
    double pos = -500;
    double pid_max_power = 1;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField vf;
    //Path path;

    State state = State.pid;
    Lift lift;
    Arm arm;
    HorizontalSlides intakeSlides;
    Intake intake;

    Alliance alliance = Alliance.red;

    enum State {
        pid,
        deposit_sample,
        setup_intake_sample,
        intake_sample,
        park
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void init() {
        sample_point = new Point(16, 125.8);

        Point[][] follow_path = {{
                new Point(11, 128.7),
                new Point(38, 121.8),
                new Point(65, 125),
                new Point(76.8, 98.3),
        }};

        BCPath path = new BCPath(follow_path);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 113, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry);
        vf.setPath(path, -90, false);

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
        intakeSlides.update();

        switch(state) {
            case pid:
                intake.up();
                intake.stop();
                arm.closeClaw();

                if (timer.milliseconds() >= 300) {
                    lift.toHighBasket();
                }

                vf.pid_to_point(sample_point, 135, pid_max_power);

                if (timer.milliseconds() >= 2500) {
                    timer.reset();
                    state = State.deposit_sample;
                }

                break;

            case deposit_sample:
                intake.up();
                intake.stop();

                arm.outtakeSample();

                if (timer.milliseconds() >= 250){
                    arm.openClaw();
                }
                if (timer.milliseconds() >= 500){
                    arm.toAutoStartPosition();
                    lift.intakeSample();

                    vf.pid_to_point(sample_point, intake_angle, pid_max_power);
                }
                if (timer.milliseconds() >= 2000){
                    state = State.intake_sample;
                }

                break;

           case intake_sample:
                intakeSlides.setPosition(pos);
                intake.intake();

                if (timer.milliseconds() >= 500){
                    intake.down();
                }

                if (intake.hasCorrectSample(true)) {
                    timer.reset();
                    state = State.pid;
                }

                break;

           case park:
                vf.move();

                lift.toHighChamber();

        }
    }
}
