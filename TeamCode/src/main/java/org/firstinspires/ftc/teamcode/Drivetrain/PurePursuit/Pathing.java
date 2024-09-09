package org.firstinspires.ftc.teamcode.Drivetrain.PurePursuit;

import static org.firstinspires.ftc.teamcode.Drivetrain.PurePursuit.Utils.doubleArrayCopy;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Pathing {
    public ArrayList<Point> injectPoints(ArrayList<Point> pathPoints) {
        double spacing = 1.0;
        ArrayList<Point> newPathPoints = new ArrayList<Point>();

        newPathPoints.add(pathPoints.get(0));

        for (int i = 1; i < pathPoints.size(); i++ ) { //loop through each line segment
            Point endPoint = pathPoints.get(i);
            Point startPoint = pathPoints.get(i-1);

            Point vector = new Point(endPoint.x - startPoint.x, endPoint.y - startPoint.y);
            double magnitude = Math.hypot(vector.x, vector.y);

            double numPointsThatFit = Math.ceil(magnitude / spacing);

            Point spacingVector = new Point(vector.x/magnitude * spacing, vector.y/magnitude * spacing);

            for (int j=0; j<numPointsThatFit; j++) {
                newPathPoints.add(new Point(startPoint.x + spacingVector.x * j, startPoint.y + spacingVector.y * j));
            }
        }

        newPathPoints.add(pathPoints.get(pathPoints.size()-1));

        return newPathPoints;
    }

    public ArrayList<Point> smoother(ArrayList<Point> pathPoints, double a, double b, double tolerance){
        double[][] path = new double[pathPoints.size()][2];

        for (int i=0; i<pathPoints.size()-1; i++) {
            path[i][0] = pathPoints.get(i).x;
            path[i][1] = pathPoints.get(i).y;
        }

        //copy array
        double[][] newPath = doubleArrayCopy(path);
        double change = tolerance;
        while(change >= tolerance)
        {
            change = 0.0;
            for(int i=1; i<path.length-1; i++)
                for(int j=0; j<path[i].length; j++)
                {
                    double aux = newPath[i][j];
                    newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b *
                            (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }

        ArrayList<Point> smoothPathPoints = new ArrayList<Point>();
        for (int i=0; i<newPath.length-1; i++) {
            smoothPathPoints.add(new Point(newPath[i][0], newPath[i][1]));
        }
        return smoothPathPoints;
    }


}
