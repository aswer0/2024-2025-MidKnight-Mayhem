package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@TeleOp
public class claw extends LinearOpMode {
    Servo ps;
    RevColorSensorV3 cs;
    private double redValue;
    private double blueValue;
    private double greenValue;

    @Override
    public void runOpMode(){
        ps = hardwareMap.get(Servo.class,"servo");
        cs = hardwareMap.get(RevColorSensorV3.class, "color");
        waitForStart();
        while(opModeIsActive()) {
            redValue = cs.red();
            blueValue = cs.blue();
            greenValue = cs.green();
            telemetry.addData("R", redValue);
            telemetry.addData("B", blueValue);
            telemetry.addData("G", greenValue);
            telemetry.update();
            ps.setPosition(0.4);
            if((redValue >= 609 || blueValue >= 609) && ((redValue >= blueValue && redValue >= greenValue) || (blueValue >= redValue && blueValue >= greenValue))){
                ps.setPosition(0.7);
                if(gamepad1.circle){
                    ps.setPosition(0.4);
                }
            }
        }
    }
}