package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotorExCaching {
    DcMotorEx motor;

    double lastPower = 0;
    double threshold;

    public DcMotorExCaching(String motorName, HardwareMap hardwareMap, double threshold) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.threshold = threshold;
    }

    public void setPower(double power) {
        if (Math.abs(power - lastPower) >= this.threshold) {
            motor.setPower(power);
            lastPower = power;
        }
    }

}
