package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

import java.util.List;

@Config
@Disabled
@TeleOp
public class PIDToPoint extends OpMode {
    public static double xp = 0.022, xi = 0, xd = 0;
    public static double yp = 0.022, yi = 0, yd = 0;
    public static double hp = 0.011, hi = 0, hd = 0.0001;

    public static double targetX = 10.75;
    public static double targetY = 30.25;
    public static double targetHeading = 0;

    public static double speed = 1;

    Odometry odometry;
    WheelControl drive;

    List<LynxModule> allHubs;

    PIDController xPid;
    PIDController yPid;
    PIDController headingPid;

    FtcDashboard dashboard;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 10.75, 30.25, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);
        drive.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        xPid = new PIDController(xp, xi, xd);
        yPid = new PIDController(yp, yi, yd);
        headingPid = new PIDController(hp, hi, hd);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        odometry.update();
        double xPower = xPid.calculate(odometry.getxPos(), targetX);
        double yPower = yPid.calculate(odometry.getyPos(), targetY);
        double turnPower = headingPid.calculate(Math.toDegrees(odometry.getHeading()), targetHeading);

        drive.drive(xPower, -yPower, -turnPower, -odometry.getHeading(), speed);

        telemetry.addData("xPos", odometry.getxPos());
        telemetry.addData("yPos", odometry.getyPos());
        telemetry.addData("heading", Math.toDegrees(odometry.getHeading()));
        telemetry.addData("xTarget", targetX);
        telemetry.addData("yTarget", targetY);
        telemetry.addData("headingTarget", targetHeading);
        telemetry.addData("xPower", xPower);
        telemetry.addData("yPower", yPower);

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setTranslation(-72, 72)
                .setRotation(Math.toRadians(-90))
                .setStroke("blue")
                .strokeCircle(odometry.getxPos(), odometry.getyPos(), 15/2)
                .strokeLine(odometry.getxPos(), odometry.getyPos(),odometry.getxPos() + 15/2*Math.cos(odometry.getHeading()), odometry.getyPos() + 15/2*Math.sin(odometry.getHeading()));
        dashboard.sendTelemetryPacket(packet);
    }
}
