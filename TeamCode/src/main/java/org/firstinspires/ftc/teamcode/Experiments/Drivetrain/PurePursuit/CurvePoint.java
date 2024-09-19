package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.PurePursuit;

import org.opencv.core.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpd;
    public double turnSpd;
    public double followDist;
    public double pointLength;
    public double slowDownTurnDist;
    public double slowDownTurnAmount;
    double followAngle;

    public CurvePoint(double x, double y, double moveSpd, double turnSpd, double followDist, double slowDownTurnDist, double slowDownTurnAmount, double followAngle) {
        this.x = x;
        this.y = y;
        this.moveSpd = moveSpd;
        this.turnSpd = turnSpd;
        this.followDist = followDist;
        this.slowDownTurnDist = slowDownTurnDist;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.followAngle = followAngle;
    }

    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpd = thisPoint.moveSpd;
        turnSpd = thisPoint.turnSpd;
        followDist = thisPoint.followDist;
        slowDownTurnDist = thisPoint.slowDownTurnDist;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        pointLength = thisPoint.pointLength;
        followAngle = thisPoint.followAngle;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    public void setPoint (Point point) {
        x = point.x;
        y = point.y;
    }
}

