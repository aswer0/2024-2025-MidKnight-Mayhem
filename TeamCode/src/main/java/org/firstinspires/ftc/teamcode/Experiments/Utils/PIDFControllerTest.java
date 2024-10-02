package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class PIDFControllerTest extends OpMode {
    DcMotorEx motor;
    FtcDashboard dashboard;
    PIDFController controller = new PIDFController(coefficients);
    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.001,0,0,0);
    public static int target = 10;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, "m1");

    }
    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", motor.getCurrentPosition());
        packet.put("Target", target);
        dashboard.sendTelemetryPacket(packet);
    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        motor.setPower(controller.update(motor.getCurrentPosition() - target));
        packet.put("Position", motor.getCurrentPosition());
        packet.put("Target", target);
        dashboard.sendTelemetryPacket(packet);
    }
}
