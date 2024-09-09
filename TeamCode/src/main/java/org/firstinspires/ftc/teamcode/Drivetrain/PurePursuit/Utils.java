package org.firstinspires.ftc.teamcode.Drivetrain.PurePursuit;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Utils {
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2*Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2*Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint1.y - linePoint2.y) / (linePoint1.x - linePoint2.x);

        //making center of circle = origin
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticA = 1.0 + m1*m1;
        double quadraticB = (2.0 * m1 * y1) - (2.0 * m1*m1 * x1);
        double quadraticC = (m1*m1 * x1*x1) - (2.0*y1*m1*x1) + (y1*y1) - (radius*radius);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(quadraticB*quadraticB - 4.0*quadraticA*quadraticC)) / (2 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //reset offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = Math.min(linePoint1.x, linePoint2.x);
            double maxX = Math.max(linePoint1.x, linePoint2.x);

            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(quadraticB*quadraticB - 4.0*quadraticA*quadraticC)) / (2 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            //reset offset
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception e) {

        }
        return allPoints;
    }

    private static Point PointLineIntersection(Point robotLocation, Point startLine, Point endLine) {
        if (endLine.x - startLine.x == 0) { //vertical line
            return new Point (startLine.x, robotLocation.y);
        } else if (endLine.y - startLine.y == 0) { //horizontal line
            return new Point(robotLocation.x, startLine.y);
        } else {
            double m1 = (endLine.y - startLine.y) / (endLine.x - startLine.x);
            double m0 = (startLine.x - endLine.x) / (endLine.y - startLine.y);

            double x = (-(robotLocation.x * m0) + robotLocation.y + (startLine.x * m1) - startLine.y) / (m1 - m0);
            double y = m1 * (x - startLine.x) + startLine.y;
            return new Point(x, y);
        }

    }

    public static double[][] doubleArrayCopy(double[][] arr)
    {

        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for(int i=0; i<arr.length; i++)
        {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for(int j=0; j<arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }

        return temp;

    }

    private static Point PointLineIntersection(Point robotLocation, CurvePoint startLine, CurvePoint endLine) {
        if (endLine.x - startLine.x == 0) { //vertical line
            return new Point (startLine.x, robotLocation.y);
        } else if (endLine.y - startLine.y == 0) { //horizontal line
            return new Point(robotLocation.x, startLine.y);
        } else {
            double m1 = (endLine.y - startLine.y) / (endLine.x - startLine.x);
            double m0 = (startLine.x - endLine.x) / (endLine.y - startLine.y);

            double x = (-(robotLocation.x * m0) + robotLocation.y + (startLine.x * m1) - startLine.y) / (m1 - m0);
            double y = m1 * (x - startLine.x) + startLine.y;
            return new Point(x, y);
        }

    }

    public static CurvePoint nextPoint(Point robotLocation, ArrayList<CurvePoint> allPoints) {
        CurvePoint nextPoint = allPoints.get(0);
        double closestDist = 1000000;

        for (int i = 0; i < allPoints.size() - 1; i ++) {
            CurvePoint startLine = allPoints.get(i);
            CurvePoint endLine = allPoints.get(i+1);

            Point intersection = PointLineIntersection(robotLocation, startLine, endLine);

            double dist = Math.hypot(robotLocation.x - intersection.x, robotLocation.y - intersection.y);

            if (dist < closestDist) {
                if ((intersection.x <= startLine.x && intersection.x >= endLine.x) || (intersection.x >= startLine.x && intersection.x <= endLine.x)) {
                    closestDist = dist;
                    nextPoint = endLine;
                }
            }
        }

        return nextPoint;
    }
}
