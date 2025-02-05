package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

@Disabled
@Config
@TeleOp
public class OdometryTest  extends OpMode {
    TelemetryPacket packet;

    Odometry odometry;
    WheelControl drive;

    @Override
    public void init() {
        packet = new TelemetryPacket();

        odometry = new Odometry(hardwareMap, 0, 0, 0, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);

        //bulk read
        //List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        //for (LynxModule hub : allHubs) {
        //    hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //}
    }

    @Override
    public void loop() {
        odometry.update();

        telemetry.addData("xPos", odometry.getxPos());
        telemetry.addData("yPos", odometry.getyPos());
        telemetry.addData("heading", Math.toDegrees(odometry.getHeading()));

        telemetry.addLine();

        telemetry.addData("left encoder", odometry.leftEncoder.getCurrentPosition());
        telemetry.addData("right encoder", odometry.rightEncoder.getCurrentPosition());
        telemetry.addData("front encoder", odometry.horizontalEncoder.getCurrentPosition());
    }
}
