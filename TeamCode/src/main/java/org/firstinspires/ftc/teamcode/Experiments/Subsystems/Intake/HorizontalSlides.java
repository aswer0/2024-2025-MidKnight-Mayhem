package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// todo PID
public class HorizontalSlides {
    public DcMotorEx horizontalSlidesMotor;

    public HorizontalSlides (HardwareMap hardwareMap) {
        horizontalSlidesMotor = hardwareMap.get(DcMotorEx.class,"horizontalSlidesMotor");
    }
}
