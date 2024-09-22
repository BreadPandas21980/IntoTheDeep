package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.ArrayList;
import java.util.List;

public class DrivetrainTest {

    public Vector2 lastClosestPoint;
    public double trackWidth = 12; //in inches, should be a little larger than real one (turning scrub)

    public double leftWheelVel;
    public double rightWheelVel;
    public static double kV = 0;
    public static double kA = 0;
    public static double kP = 0;



    public Vector2[] injection(double spacing, Vector2[] path) {
        List<Vector2> newPoints = new ArrayList<>(path.length);
        Vector2 startPoint = new Vector2(path[0].x, path[0].y);
        for (int i = 1; i < path.length; i++) {
            Vector2 vector = new Vector2(path[i].x, path[i].y);
            double numOfPointsThatFit = Math.ceil(vector.mag() / spacing);
            vector.norm();
            vector.mult(spacing);
            for(int j = 0; j < numOfPointsThatFit; j++) {
                newPoints.add(new Vector2(startPoint.x + vector.x * j, startPoint.y + vector.y * j));
            }
            startPoint = new Vector2(path[i].x, path[i].y);
        }
        return newPoints.toArray(new Vector2[0]);
    }


    //https://www.desmos.com/calculator/4pv9d7giqe
    //smoothing
    //This path goes from {1,1} -> {5,1} -> {9,12} -> {12, 9} -> {15,6} -> {15,4}
    public Vector2[] smoother(Vector2[] path, double a, double b, double tolerance){
        //copy path into 2d array
        double[][] pathCopy = Vector2.oneDtoTwoD(path);
        double[][] newPath = Vector2.oneDtoTwoD(path);

        double change = tolerance;
        while(change >= tolerance)
        {
            change = 0.0;
            for(int i=1; i<pathCopy.length-1; i++)
                for(int j=0; j < pathCopy[i].length; j++)
                {
                    double aux = newPath[i][j];
                    newPath[i][j] += a * (pathCopy[i][j] - newPath[i][j]) + b *
                            (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }


        return Vector2.twoDtoOneD(newPath);
    }

    public Vector2 findClosestPoint(Vector2[] path) {
        double closestDist = Integer.MAX_VALUE;
        int closestIndex = -1;
        int indexLastClosestPoint = -1;
        int startIndex = 0;

        //start from the next point so robot never goes backwards
        for(int i = 0; i < path.length; i++) {
            if( (lastClosestPoint.equals(path[i])) ){
                indexLastClosestPoint = i;
                break;
            }
        }
        if(indexLastClosestPoint != -1) {
            startIndex = indexLastClosestPoint + 1;
        }
        for(int j = startIndex; j < path.length; j++) {
            if(Vector2.distanceAlongPath(path, path[j]) < closestDist) {
                closestDist = Vector2.distanceAlongPath(path, path[j]);
                closestIndex = j;
            }
        }
        return path[closestIndex];
    }

    //lookahead radius 12-25, smaller if curvy
    //if null use last loookahead point
    public Vector2 lookAheadPoint(Pose2d robotPose, Vector2 lineStart, Vector2 lineEnd, double lookAheadRadius) {

        //changes line to a direction vector (moves it to origin)
        Vector2 direction = new Vector2(lineEnd.x - lineStart.x, lineEnd.y - lineStart.y);
        //centers the circle (the one around the robot) at the origin for easier calculation
        //gets the vector from the center of the robot to the start of the ray
        Vector2 robotToStart = new Vector2(lineStart.x - robotPose.x, lineStart.y - robotPose.y);

        //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899
        double a = Vector2.dotProduct(direction, direction);
        double b = Vector2.dotProduct(robotToStart, direction);
        double c = Vector2.dotProduct(robotToStart, robotToStart) - Math.pow(lookAheadRadius, 2);

        double discriminant = Math.pow(b, 2) - 4 * a * c;

        if (discriminant < 0) {
            return null; //no intersections :(
            //completely missed the circle
        } else {

            //either solution might be on/off line so test both
            //t1 always smaller bc both discriminant and a will be non-negative
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b + discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            //https://www.desmos.com/calculator/knswyxcuhk
            //im confused lol

            // 3x HIT cases:
            //          -o->             --|-->  |            |  --|->
            // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

            // 3x MISS cases:
            //       ->  o                     o ->              | -> |
            // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

            //bounded between 0 and 1 to make sure it intersects
            //
            if (t1 >= 0 && t1 <= 1) {
                direction.mult(t1);
                Vector2 lookPoint = Vector2.add(direction, new Vector2(lineStart.x, lineStart.y));
                lookPoint.lookAheadRadius = lookAheadRadius;
                return lookPoint;
            }
            if (t2 >= 0 && t2 <= 1) {
                direction.mult(t2);
                Vector2 lookPoint = Vector2.add(direction, new Vector2(lineStart.x, lineStart.y));
                lookPoint.lookAheadRadius = lookAheadRadius;
                return lookPoint;
            } else {
                return null;
            }
        }
    }

    public double findArcCurvature(Vector2 lookAheadPoint, Pose2d robotPose) {
        double a = -Math.tan(robotPose.getHeading());
        double b = 1;
        double c = Math.tan(robotPose.getHeading()) * robotPose.getX() - robotPose.getY();
        double x = Math.abs(a * lookAheadPoint.x + b * lookAheadPoint.y + c) / Math.sqrt(a*a + b*b);
        double curvature = 2 * x / Math.pow(lookAheadPoint.lookAheadRadius, 2);
        //if point on right, sign positive
        //if point on left of robot, sign negative
        //cross product of RB RL vectors
        double side = Math.signum(
                Math.sin(robotPose.getHeading()) * (lookAheadPoint.x - robotPose.getX())
                - Math.cos(robotPose.getHeading()) * (lookAheadPoint.y - robotPose.getY())
        );
        double signedCurvature = side * curvature;

        return  signedCurvature;
    }

    public void findWheelVelocities(Vector2[] path, int index, Pose2d robotPose, double lookAheadRadius) {
        Vector2.findTargetVelocityAtPoint(path);
        double targVelocity = path[index].targetVelocity;
        Vector2 lineStart = new Vector2(path[index].x, path[index].y);
        Vector2 lineEnd = new Vector2(path[index + 1].x, path[index + 1].y);
        Vector2 lookAheadPoint = lookAheadPoint(robotPose, lineStart, lineEnd, lookAheadRadius);

        double arcCurvature = findArcCurvature(lookAheadPoint, robotPose);

        //use mecanum kinematics
        //https://gm0.org/en/latest/docs/software/concepts/kinematics.html

        //skidsteer kinematics = tank drive
        leftWheelVel = targVelocity * (2 + arcCurvature * trackWidth) / 2;
        rightWheelVel = targVelocity * (2 - arcCurvature * trackWidth) / 2;

    }


}
