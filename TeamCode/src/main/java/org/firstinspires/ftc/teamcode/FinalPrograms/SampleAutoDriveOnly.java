//package org.firstinspires.ftc.teamcode.FinalPrograms;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.VectorField;
////import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.CompositePath;
//import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
//import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
////import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
////import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
////import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
////import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
////import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
//import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
////import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
//import org.opencv.core.Point;
//
//@Autonomous
//@Config
//public class SampleAutoDriveOnly extends OpMode {
//    double intake_slide_extend = -450;
//    double corner_offset = 18;
//    Point start_point;
//    double get_sample_x, get_sample_y;
//    Point deposit_sample_target;
//    Point park_target;
//
//    int deposit_state;
//    ElapsedTime timer;
//
//    Odometry odometry;
//    WheelControl wheelControl;
//    //BCPath bcpath;
//    VectorField vf;
//
//    State state = State.pid;
//
//    Alliance alliance = Alliance.red;
//
//    enum State {
//        pid,
//        deposit_sample,
//        move_to_intake_sample,
//        intake_sample,
//        transfer_sample,
//        park
//    }
//
//    @Override
//    public void start() {
//        timer.reset();
//    }
//
//    @Override
//    public void init() {
//        start_point = new Point(7.875, 96);
//        get_sample_x = 25;
//        get_sample_y = 127;
//        deposit_state = 0;
//        deposit_sample_target = new Point(corner_offset, 144-corner_offset);
//        park_target = new Point(20, 40);
//
//        timer = new ElapsedTime();
//
//        odometry = new Odometry(hardwareMap, 0, start_point.x, start_point.y, "OTOS");
//        wheelControl = new WheelControl(hardwareMap, odometry);
//        //bcpath = new BCPath(follow_path);
//        vf = new VectorField(wheelControl, odometry);
//
//        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.left_bumper) {
//            alliance = Alliance.blue;
//        } else if (gamepad1.right_bumper) {
//            alliance = Alliance.red;
//        }
//
//        telemetry.addData("Alliance", alliance);
//    }
//
//    @Override
//    public void loop() {
//        odometry.opt.update();
//        TelemetryPacket telemetry = new TelemetryPacket();
//        telemetry.put("state", state);
//        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
//
//        switch(state) {
//            case pid:
//                vf.pid_to_point(deposit_sample_target, 135, 0.5);
//
//                if (vf.dist_to_end() <= 1 || timer.milliseconds() >= 5000) {
//                    timer.reset();
//                    state = State.deposit_sample;
//                }
//
//                break;
//
//            case deposit_sample:
//                wheelControl.stop();
//
//                if (timer.milliseconds() >= 300) {
//                    timer.reset();
//                    deposit_state++;
//                    if (deposit_state <= 3) {
//                        state = State.move_to_intake_sample;
//                    } else {
//                        state = State.park;
//                    }
//                }
//
//                break;
//
//            case move_to_intake_sample:
//                vf.pid_to_point(new Point(get_sample_x, get_sample_y), -180, 0.5);
//
//                if (vf.dist_to_end() <= 2 || timer.milliseconds() >= 4000) {
//                    timer.reset();
//                    state = State.intake_sample;
//                }
//
//                break;
//
//            case intake_sample:
//                wheelControl.stop();
//
//                if (timer.milliseconds() >= 2000) {
//                    timer.reset();
//                    get_sample_y += 10;
//                    state = State.transfer_sample;
//                }
//
//                break;
//
//            case transfer_sample:
//                wheelControl.stop();
//
//                if (timer.milliseconds() >= 1300) {
//                    timer.reset();
//                    state = State.pid;
//                }
//
//                break;
//
//            case park:
//                vf.pid_to_point(park_target, 0, 0.5);
//                break;
//        }
//    }
//}
