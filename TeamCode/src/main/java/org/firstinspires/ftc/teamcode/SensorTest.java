package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorTest extends OpMode {
    FtcDashboard dashboard;

    DistanceSensor distanceSensor1;
    DistanceSensor distanceSensor2;

    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;

    public void init() {
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "d1");

        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "d2");

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "c1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "c2");

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("d1 Distance", distanceSensor1.getDistance(DistanceUnit.INCH));
        packet.put("d2 Distance", distanceSensor2.getDistance(DistanceUnit.INCH));

        packet.put("c1 R", colorSensor1.getNormalizedColors().red);
        packet.put("c1 G", colorSensor1.getNormalizedColors().green);
        packet.put("c1 B", colorSensor1.getNormalizedColors().blue);

        packet.put("c2 R", colorSensor2.getNormalizedColors().red);
        packet.put("c2 G", colorSensor2.getNormalizedColors().green);
        packet.put("c2 B", colorSensor2.getNormalizedColors().blue);
        dashboard.sendTelemetryPacket(packet);

    }
}
