package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;

@TeleOp
@Config
public class PIDFControllerTest extends OpMode {
    HorizontalSlides lift;
    FtcDashboard dashboard;
    public static int target = 10;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        lift = new HorizontalSlides(hardwareMap);

    }
    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", lift.horizontalSlidesMotor.getCurrentPosition());
        packet.put("Target", target);
        dashboard.sendTelemetryPacket(packet);
    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        lift.update();
        lift.setPosition(target);
        packet.put("Position", lift.horizontalSlidesMotor.getCurrentPosition());
        packet.put("Target", target);
        dashboard.sendTelemetryPacket(packet);
    }
}
