package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

@Config
@TeleOp
public class TwoWheelOdometryTest extends OpMode {
    TelemetryPacket packet;

    TwoWheelOdometry odometry;
    WheelControl drive;

    @Override
    public void init() {

        odometry = new TwoWheelOdometry(hardwareMap, 0, 0, 0, "BR", "BL");

        //bulk read
        //List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        //for (LynxModule hub : allHubs) {
        //    hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //}
    }

    @Override
    public void loop() {
        odometry.update();
        packet = new TelemetryPacket();
        packet.put("xPos", odometry.getXPos());
        packet.put("yPos", odometry.getYPos());
        packet.put("d_x", odometry.d_x);
        packet.put("d_y", odometry.d_y);
        packet.put("heading", Math.toDegrees(odometry.getHeadingRad()));
        packet.put("vertical encoder", odometry.vertical_encoder.getCurrentPosition());
        packet.put("horizontal encoder", odometry.horizontal_encoder.getCurrentPosition());
        (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
    }
}
