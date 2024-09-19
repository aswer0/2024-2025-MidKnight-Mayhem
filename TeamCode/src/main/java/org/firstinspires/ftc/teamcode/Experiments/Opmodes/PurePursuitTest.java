package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.PurePursuit.Movement;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class PurePursuitTest extends OpMode {
    private static final double START_X = 10.75;
    private static final double START_Y = 30.25;

    public static double moveSpd = 0.9;
    public static double turnSpd = 1;
    public static double lastSpd = 0.7;
    public static double followDist = 16;


    Odometry odometry;
    WheelControl drive;
    Movement movement;

    ArrayList<Point> pastRobotPos;
    ElapsedTime timer;

    List<LynxModule> allHubs;

    FtcDashboard dashboard;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, START_X, START_Y, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);
        movement = new Movement(hardwareMap, odometry, drive);

        pastRobotPos = new ArrayList<>();
        timer = new ElapsedTime();
        timer.reset();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void start() {
        movement.resetTimer();
    }

    @Override
    public void loop() {
        odometry.update();

        ArrayList<CurvePoint> pathPoints = new ArrayList<>();

        pathPoints.add(new CurvePoint(33, 40, moveSpd, turnSpd, followDist, 12, 0.1, 0));
        pathPoints.add(new CurvePoint(20, 72, moveSpd, turnSpd, followDist, 10, 0.3, 0));
        pathPoints.add(new CurvePoint(48, 100, 0.6, turnSpd, followDist, 1, 1, 0));

        CurvePoint followMe = movement.followCurve(pathPoints, 4, lastSpd, 45);
        //movement.goToPoint(10, 20,0.4,0,0.5);


        telemetry.addData("xPos", odometry.getxPos());
        telemetry.addData("yPos", odometry.getyPos());
        telemetry.addData("heading", Math.toDegrees(odometry.getHeading()));

        if (timer.milliseconds() > 50) {
            pastRobotPos.add(new Point(odometry.getxPos(), odometry.getyPos()));
            timer.reset();
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay() //draw robot
                .setTranslation(-72, 72)
                .setRotation(Math.toRadians(-90))
                .setStroke("blue")
                .strokeCircle(odometry.getxPos(), odometry.getyPos(), 15/2)
                .strokeLine(odometry.getxPos(), odometry.getyPos(),odometry.getxPos() + 15/2*Math.cos(odometry.getHeading()), odometry.getyPos() + 15/2*Math.sin(odometry.getHeading()));

        packet.fieldOverlay() //draw target path
                .setFill("red")
                .setStroke("orange")
                .fillCircle(START_X, START_Y, 2)
                .strokeLine(START_X, START_Y, pathPoints.get(0).x, pathPoints.get(0).y);
        for (int i=0; i<pathPoints.size(); i++) {
            packet.fieldOverlay()
                    .fillCircle(pathPoints.get(i).x, pathPoints.get(i).y, 2);
        }
        for (int i=0; i<pathPoints.size()-1; i++) {
            packet.fieldOverlay()
                    .strokeLine(pathPoints.get(i).x, pathPoints.get(i).y, pathPoints.get(i + 1).x, pathPoints.get(i + 1).y);
        }

        packet.fieldOverlay() //draw follow point
                .setFill("white")
                .fillCircle(followMe.x, followMe.y, 1.25);


        for (int i=0; i<pastRobotPos.size(); i++) { //draw robot path
            Point pos = pastRobotPos.get(i);
            packet.fieldOverlay()
                    .setFill("yellow")
                    .fillCircle(pos.x, pos.y, 0.75);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
