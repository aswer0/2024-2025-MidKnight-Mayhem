package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Utils;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
public class RedRightGVF extends OpMode {
    Point specimen_target;
    ElapsedTime timer;
    boolean start_hang_path = false;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField hang_gvf;

    State state = State.startPID;
    Lift lift;
    Manipulator manipulator;

    Intake intake;

    enum State {
        startPID,
        getSpecimen,
        followPath,
        deposit
    }

    @Override
    public void init() {
        Point[][] hang_path_cp = {
                {
                        new Point(22, 72),
                        new Point(5, -10),
                        new Point(64, 45)
                }
        };
        BCPath hang_path = new BCPath(hang_path_cp);
        specimen_target = new Point(35.5, 66);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        hang_gvf = new VectorField(wheelControl, odometry, hang_path, 90);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);

        intake = new Intake(hardwareMap, new Sensors(hardwareMap, telemetry));

        manipulator.closeClaw();
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        //wheelControl.drive(0, 0, 1, 0, 0.15);
    //}
        odometry.opt.update();
        TelemetryPacket telemetry = new TelemetryPacket();

        switch (state){
            case startPID:
                manipulator.closeClaw();
                lift.toHighChamber();
                hang_gvf.move_to_point(specimen_target, 0, 0.5);

                if (Utils.dist(odometry.opt.get_pos(), specimen_target) < 0.2){
                    timer.reset();
                    state = State.deposit;
                }

                break;

            case deposit:
                lift.setPosition(550);
                if (timer.milliseconds() >= 250){
                    manipulator.openClaw();
                }

                break;

        }

        telemetry.put("X position: ", odometry.opt.get_x());
        telemetry.put("Y position: ", odometry.opt.get_y());
        telemetry.put("Heading: ", odometry.opt.get_heading());
        telemetry.put("D: ", hang_gvf.D);
        telemetry.put("Speed: ", hang_gvf.speed);
        telemetry.put("Velocity: ", hang_gvf.velocity);
        telemetry.put("Error: ", hang_gvf.error);
        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
    }
}