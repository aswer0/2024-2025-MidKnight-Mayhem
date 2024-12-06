package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class SpecimenAuto extends OpMode {
    public static double pos = -470;
    public static double target_angle = 135;
    public static double sample_x = 24.5;
    public static double sample_y = 37;
    public static double power = 1;

    Point target;
    Point get_specimen_target;
    Point get_sample_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    SpecimenAuto.State state = State.pid;
    Lift lift;
    Manipulator manipulator;
    HorizontalSlides horizontalSlides;
    Intake intake;

    double deposit_state = 0;
    int intakeState = 0;

    enum State {
        pid,
        goToSpecimen,
        pickupSpecimen,
        intakeSample,
        manage,
        deposit,
        gotoSample,
        manageDepositState,
        spitSample
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(7.875, 21.3),
                new Point(48, 21.3),
                new Point(3.7, 28.6),
                new Point(43.6, 47.3),
                new Point(16.5, 72.4),
                new Point(23, 65),
                new Point(36.2, 66)
        };

        target = new Point(36.2, 72);
        get_specimen_target = new Point(12.875, 28);
        get_sample_target = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 180, power);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);
        intake = new Intake(hardwareMap);
        horizontalSlides = new HorizontalSlides(hardwareMap);

        manipulator.closeClaw();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        odometry.opt.update();

        switch (state){
            case manageDepositState:
                intake.stop();
                intake.up();
                if (deposit_state == 0 || deposit_state == 1){
                    state = State.pid;
                }
                if (deposit_state == 2){
                    timer.reset();
                    state = State.gotoSample;
                }
                break;

            case pid:
                intake.up();
                manipulator.closeClaw();

                if (timer.milliseconds() >= 100 || deposit_state==0){
                    lift.toHighChamber();
                }

                path.follow_pid_to_point(target, 0);

                if (sensors.atChamber() && odometry.opt.get_heading()>-50 && odometry.opt.get_heading()<50){
                    timer.reset();
                    state = State.deposit;
                }

                if (timer.milliseconds()>5000) {
                    timer.reset();
                    state = State.deposit;
                }

                break;

            case deposit:
                intake.up();
                lift.setPosition(525);
                if (timer.milliseconds() >= 310){
                    manipulator.openClaw();
                    if (deposit_state == 0){
                        odometry.opt.setPos(odometry.opt.get_x(), odometry.opt.get_y(), -5);
                    }

                    deposit_state++;
                    state = State.manage;
                }

                break;

            case goToSpecimen:
                intake.up();
                path.follow_pid_to_point(get_specimen_target, 180);
                lift.toLowChamber();

                if (path.at_point(get_specimen_target, 4)){
                    state = State.pickupSpecimen;
                }

                break;

            case manage:
                if (deposit_state == 2) {
                    state = State.manageDepositState;
                }
                else{
                    state = State.goToSpecimen;
                }

            case pickupSpecimen:
                intake.up();

                if (sensors.get_front_dist() >= 2.5){
                    wheelControl.drive(-0.3, 0, 0, 0, 0.7);
                }
                else{
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    manipulator.closeClaw();

                    timer.reset();
                    target.y += 0.25;
                    target.x -= 1.25;
                    //deposit_state++;
                    state = State.pid;
                }

                break;

            case gotoSample:
                lift.toLowChamber();
                intake.up();
                path.follow_pid_to_point(get_sample_target, target_angle);

                if (path.at_point(get_sample_target, 1) || timer.milliseconds()>1500){
                    wheelControl.drive(0, 0, 0, 0, 0);
                    timer.reset();
                    state = State.intakeSample;
                }

                break;

            case intakeSample:
                lift.toLowChamber();
                if (timer.milliseconds() >= 500){
                    horizontalSlides.setPosition(pos);
                }
                intake.intake();
                intake.down();

                if (intakeState == 0 && Math.abs(horizontalSlides.horizontalSlidesMotor.getCurrentPosition()-pos) <= 15){
                    intakeState++;
                }
                if (intakeState > 0 && timer.milliseconds()>2000) {
                    deposit_state++;
                    timer.reset();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                Point p = new Point(odometry.opt.get_x(), odometry.opt.get_y());
                path.follow_pid_to_point(p, 35);

                if (timer.milliseconds() < 500){
                    horizontalSlides.setPosition(-200);
                } else{
                    horizontalSlides.setPosition(pos);
                }

                if (timer.milliseconds() >= 1000){
                    intake.reverse();
                } if (timer.milliseconds() >= 2000){
                    intake.up();
                    intake.stop();
                }


        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Deposit State: ", deposit_state);
        telemetry.addData("Front Distance", sensors.get_front_dist());

        lift.update();
        horizontalSlides.update();
        telemetry.update();
    }
}
