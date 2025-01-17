package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class TwoSpecimenAuto extends OpMode {
    public static double sample_x = 24.5;
    public static double sample_y = 37;
    public static double pos = 550;

    public static double power = 0.7;

    public static double target_x = 36.5; //36.2
    public static double target_y = 72;

    Point target;
    Point get_specimen_target;
    Point get_sample_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    TwoSpecimenAuto.State state = TwoSpecimenAuto.State.pid;
    Lift lift;
    Manipulator manipulator;
    Intake intake;
    Arm arm;

    enum State {
        pid,
        goToSpecimen,
        pickupSpecimen,
        deposit,
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(40.2, 68.7),
                new Point(18.8, 14.7),
                new Point(65.3, 55.2),
                new Point(60.9, 23.3),
        };

        target = new Point(target_x, target_y);
        get_specimen_target = new Point(12.875, 28);
        get_sample_target = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 180, power);
        arm = new Arm(hardwareMap);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);
        intake = new Intake(hardwareMap, sensors);

        manipulator.closeClaw();
        arm.toAutoStartPosition();
        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        intake.sweeperIn();
        odometry.opt.update();

        switch (state) {
            case pid:
                intake.up();
                manipulator.closeClaw();
                lift.setPosition(1300);

                path.follow_pid_to_point(target, 0);

                if (sensors.atChamber() && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                    timer.reset();
                    state = State.deposit;
                }

                if (timer.milliseconds() > 5000) {
                    timer.reset();
                    state = State.deposit;
                }

                break;

            case deposit:
                intake.up();
                lift.setPosition(pos);

                if (timer.milliseconds() >= 310) {
                    manipulator.openClaw();
                }
                if (timer.milliseconds() >= 400){
                    timer.reset();
                    state = State.goToSpecimen;
                }

                break;

            case goToSpecimen:
                intake.up();
                path.follow_pid_to_point(get_specimen_target, 179);

                if (timer.milliseconds() > 500){
                    lift.setPosition(90);
                }

                if (path.at_point(get_specimen_target, 5)) { //4
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                intake.up();

                if (sensors.get_front_dist() >= 2.5) {
                    wheelControl.drive(-0.3, 0, 0, 0, 0.7);
                } else {
                    wheelControl.drive(0, 0, 0, 0, 0);
                    manipulator.closeClaw();

                    timer.reset();
                    target.y -= 2;
                    state = State.pid;
                }

                break;
        }        
                
        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Front Distance", sensors.get_front_dist());

        lift.update();
        telemetry.update();
    }
    
}
