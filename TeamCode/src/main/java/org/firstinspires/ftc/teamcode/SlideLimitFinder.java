package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SlideLimitFinder extends OpMode {

    DcMotorEx motor;
    FtcDashboard dashboard;
    int minPosition = Integer.MAX_VALUE;
    int maxPosition = Integer.MIN_VALUE;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "m1");
        dashboard = FtcDashboard.getInstance();
        motor.getCurrentPosition();
    }

    @Override
    public void loop() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        int position =
                motor.getCurrentPosition();
        telemetryPacket.put("Minimum", minPosition = Math.min(minPosition, position));
        telemetryPacket.put("Maximum", maxPosition = Math.max(maxPosition, position));
        telemetryPacket.put("Current position", position);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}
