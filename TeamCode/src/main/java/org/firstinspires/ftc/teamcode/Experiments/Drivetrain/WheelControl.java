package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class WheelControl {
    public DcMotorEx BR;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx FL;
    private VoltageSensor voltageSensor;
    Odometry odometry;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public WheelControl(HardwareMap hardwareMap, Odometry odometry) {
        this.BR = hardwareMap.get(DcMotorEx.class, "BR");
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FR = hardwareMap.get(DcMotorEx.class, "FR");
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.BL = hardwareMap.get(DcMotorEx.class, "BL");
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FL = hardwareMap.get(DcMotorEx.class, "FL");
        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.BL.setDirection(DcMotorEx.Direction.REVERSE);
        this.FL.setDirection(DcMotorEx.Direction.REVERSE);

        this.odometry = odometry;
        this.voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }
    public void setPowers(double BL, double BR, double FL, double FR, double power) {
        double max = 1; // max motor power

        max = Math.max(BL, max);
        max = Math.max(BR, max);
        max = Math.max(FL, max);
        max = Math.max(FR, max); // Detect the motor with the most power
        this.BL.setPower(power * (BL/max));
        this.BR.setPower(power * (BR/max));
        this.FL.setPower(power * (FL/max));
        this.FR.setPower(power * (FR/max)); // We divide all values by the maximum one so they do not reach one.
    }

    /**
     *
     * @param forward Y component of the vector (robot oriented)
     * @param right   X component of the vector (robot oriented)
     * @param rotate  Rotation velocity (radians)
     * @param angle   The angle for where to rotate the thing. Get from odometry. (field oriented)
     */
    public void drive(double forward, double right, double rotate, double angle, double power) {
        double newX = right*Math.cos(angle) - forward*Math.sin(angle);
        double newY = right*Math.sin(angle) + forward*Math.cos(angle);

        double BLPower = newY - newX + rotate;
        double BRPower = newY + newX - rotate;
        double FLPower = newY + newX + rotate;
        double FRPower = newY - newX - rotate;
        //setPowers(BLPower, BRPower, FLPower, FRPower, power);

        double max = 1; // max motor power

        max = Math.max(BLPower, max);
        max = Math.max(BRPower, max);
        max = Math.max(FLPower, max);
        max = Math.max(FRPower, max); // Detect the motor with the most power
        if (!(BLPower==0)) {
            this.BL.setPower(power * (BLPower/max) + 0.06*BLPower/Math.abs(BLPower));
        } else {
            this.BL.setPower(0);
        }
        if (!(BRPower==0)) {
            this.BR.setPower(power * (BRPower/max) + 0.06*BRPower/Math.abs(BRPower));
        } else {
            this.BR.setPower(0);
        }if (!(FLPower==0)) {
            this.FL.setPower(power * (FLPower/max) + 0.06*FLPower/Math.abs(FLPower));
        } else {
            this.FL.setPower(0);
        }if (!(FRPower==0)) {
            this.FR.setPower(power * (FRPower/max) + 0.06*FRPower/Math.abs(FRPower));
        } else {
            this.FR.setPower(0);
        }
    }
}
