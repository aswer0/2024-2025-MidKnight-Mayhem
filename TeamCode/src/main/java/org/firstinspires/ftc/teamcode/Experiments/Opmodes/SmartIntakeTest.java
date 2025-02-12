package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;

@Config
@TeleOp
public class SmartIntakeTest extends OpMode {
    Intake intake;
    HorizontalSlides intakeSlides;
    Alliance alliance;

    @Override
    public void init() {
        intake = new Intake(hardwareMap, new Sensors(hardwareMap, telemetry));
        intakeSlides = new HorizontalSlides(hardwareMap);
        alliance = Alliance.red;
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        intakeSlides.update();
        intakeSlides.setPosition(-450);
        intake.smarterIntake(true);
    }
}