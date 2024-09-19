package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.PurePursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Movement {
    public static double xp = 0.022, xi = 0, xd = 0.001;
    public static double yp = 0.022, yi = 0, yd = 0.001;
    public static double hp = 0.011, hi = 0, hd = 0.0001;

    PIDController xPid;
    PIDController yPid;
    PIDController headingPid;

    ElapsedTime timer;

    HardwareMap hardwareMap;
    Odometry odometry;
    WheelControl drive;

    public Movement (HardwareMap hardwareMap, Odometry odometry, WheelControl drive) {
        xPid = new PIDController(xp, xi, xd);
        yPid = new PIDController(yp, yi, yd);
        headingPid = new PIDController(hp, hi, hd);

        timer = new ElapsedTime();

        this.hardwareMap = hardwareMap;
        this.odometry = odometry;
        this.drive = drive;
    }

    public void resetTimer() {timer.reset();}

    public CurvePoint followCurve(ArrayList<CurvePoint> allPoints, double slowThreshold, double lastSpd, double endAngle) {
        CurvePoint lastPoint = allPoints.get(allPoints.size()-1);

        CurvePoint nextPoint = Utils.nextPoint(new Point(odometry.getxPos(), odometry.getyPos()), allPoints);

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(odometry.getxPos(), odometry.getyPos()),
                nextPoint.followDist, nextPoint.followAngle); //change later

        double moveSpeed = Math.min(nextPoint.moveSpd, 2*timer.seconds());
        moveSpeed = Math.min(moveSpeed, moveSpeed*nextPoint.slowDownTurnAmount + (moveSpeed - moveSpeed*nextPoint.slowDownTurnAmount)*Math.hypot(nextPoint.x - odometry.getxPos(), nextPoint.y - odometry.getyPos())/nextPoint.slowDownTurnDist);

        double turnSpeed = Math.min(nextPoint.turnSpd, 3*timer.seconds());

        if (Math.hypot(Math.abs(odometry.getxPos() - lastPoint.x), Math.abs(odometry.getyPos() - lastPoint.y)) > lastPoint.followDist + slowThreshold) {
            goToPoint(followMe.x, followMe.y, moveSpeed, nextPoint.followAngle, turnSpeed);
        } else {
            pidToPoint(lastPoint.x, lastPoint.y, endAngle, lastSpd);
        }
        return followMe;
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius, double preferredAngle) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = Utils.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000;

            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - odometry.getyPos(), thisIntersection.x - odometry.getxPos());
                double deltaAngle = Math.abs(Utils.AngleWrap(angle - odometry.getHeading() + preferredAngle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void goToPoint(double x, double y, double moveSpd, double preferredAngle, double turnSpd) {
        double distToTarget = Math.hypot(x-odometry.getxPos(), y-odometry.getyPos());

        double absoluteAngleToTarget = Math.atan2(y-odometry.getyPos(), x-odometry.getxPos());

        double relativeAngleToPoint = Utils.AngleWrap(absoluteAngleToTarget - (odometry.getHeading())); //- Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(0) + preferredAngle;

        double movement_x = movementXPower * moveSpd;
        double movement_y = movementYPower * moveSpd;
        double movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpd;

        if (distToTarget < 2) {
            movement_x = 0;
            movement_y = 0;
        }
        if (distToTarget < 5) {
            movement_turn = 0;
        }

        drive.drive(movement_x, movement_y, -movement_turn, 0, 1);
    }

    public void pidToPoint(double x, double y, double angle, double speed) {
        double xPower = xPid.calculate(odometry.getxPos(), x);
        double yPower = yPid.calculate(odometry.getyPos(), y);
        double turnPower = headingPid.calculate(Math.toDegrees(odometry.getHeading()), angle);

        drive.drive(xPower, -yPower, -turnPower, -odometry.getHeading(), speed);
    }
}

