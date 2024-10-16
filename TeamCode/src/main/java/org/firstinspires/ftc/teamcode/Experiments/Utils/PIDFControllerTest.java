package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class PIDFControllerTest extends OpMode {
    DcMotorEx motor;
    FtcDashboard dashboard;
    PIDFController controller = new PIDFController(coefficients);
    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.02,0,-0.001,0.1); // Set D to -.0015 if you want smother input
    public static int target = 10;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, "FL");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", motor.getCurrentPosition());
        packet.put("Target", target);
        packet.put("Input", 0);
        dashboard.sendTelemetryPacket(packet);
    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        double input = controller.update(motor.getCurrentPosition() - target);
        motor.setPower(/*Math.abs(input) > 0.5? Math.signum(input) * 0.5 :*/ input);
        packet.put("Position", motor.getCurrentPosition());
        packet.put("Target", target);
        packet.put("Input", /*Math.abs(input) > 0.5? Math.signum(input) * 0.5 :*/ input);
        dashboard.sendTelemetryPacket(packet);
    }
}
