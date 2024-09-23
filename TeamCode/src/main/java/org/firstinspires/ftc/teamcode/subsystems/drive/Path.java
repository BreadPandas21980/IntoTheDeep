package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a path to be followed by the Pure Pursuit algorithm
 */
public class Path {
    private double spacing;
    private ArrayList<Vector2> robotPath = new ArrayList<>();
    private Vector2 endVector = new Vector2(0, 0);
    private boolean forward;

    public Path(double spacing, boolean forward) {
        this.spacing = spacing;
        this.forward = forward;
    }

    public ArrayList<Vector2> getRobotPath() {
        return robotPath;
    }

    public Vector2 getStartPoint() {
        return robotPath.get(0);
    }

    public Vector2 getEndPoint() {
        return robotPath.get(robotPath.size() - 1);
    }

    public boolean isForward() {
        return forward;
    }

    /**
     * Initializes the path
     *
     * @param maxVel   maximum robot velocity
     * @param maxAccel maximum robot acceleration
     * @param maxVelk  maximum turning velocity (between 1-5)
     */
    public void initializePath(double maxVel, double maxAccel, double maxVelk) {
        setCurvatures();
        setDistances();
        setTargetVelocities(maxVel, maxAccel, maxVelk);
    }

    /**
     * Adds a path segment to this path
     * @param start starting point
     * @param end ending point
     */
    public void addSegment(Vector2 start, Vector2 end) {
        ArrayList<Vector2> injectTemp = new ArrayList<>();
        injection(start, end, injectTemp);
        robotPath.addAll(injectTemp);
        endVector = end;
    }

    /**
     * Injects points into this path
     * @param startPoint starting point
     * @param endPoint ending point
     * @param temp temporary storage for injected points
     */
    public void injection(Vector2 startPoint, Vector2 endPoint, ArrayList<Vector2> temp) {
        Vector2 vector = Vector2.subtract(endPoint, startPoint);
        double numOfPointsThatFit = Math.ceil(vector.mag() / spacing);
        Vector2 unitVector = vector.normalize(null);
        unitVector.mult(vector.mag() / numOfPointsThatFit);
        for (int i = 0; i < numOfPointsThatFit; i++) {
            Vector2 newVector = Vector2.mult(unitVector, i, null);
            temp.add(Vector2.add(startPoint, newVector));
        }
    }
    /**
     * Smooths the path using gradient descent
     * @param a 1-b
     * @param b smoothing factor (higher = more smooth)
     * @param tolerance convergence tolerance amount (higher = less smoothing)
     */
    public void smooth(double a, double b, double tolerance) {
        ArrayList<Vector2> newPath = new ArrayList<>();
        for (Vector2 v : robotPath) {
            newPath.add(new Vector2(v));
        }
        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < robotPath.size() - 1; ++i) {
                Vector2 oldVec = robotPath.get(i);
                Vector2 currVec = newPath.get(i);
                Vector2 currVecCopy = new Vector2(currVec);
                Vector2 prevVec = newPath.get(i - 1);
                Vector2 nextVec = newPath.get(i + 1);
                currVec.x += a * (oldVec.x - currVec.x) + b * (prevVec.x + nextVec.x - 2 * currVec.x);
                currVec.y += a * (oldVec.y - currVec.y) + b * (prevVec.y + nextVec.y - 2 * currVec.y);
                change += Math.abs(currVecCopy.x - currVec.x);
                change += Math.abs(currVecCopy.y - currVec.y);
            }
        }
        ArrayList<Vector2> path = new ArrayList<>();
        for (Vector2 v : newPath)
            path.add(new Vector2(v));
        robotPath = path;
    }

    public void addLastPoint() {
        robotPath.add(endVector);
    }

    //calculations for point attributes (curvature and max velocity)

    private double calculatePathCurvature(ArrayList<Vector2> path, int pointIndex) {
        Vector2 point = new Vector2(path.get(pointIndex));
        Vector2 prevPoint = new Vector2(path.get(pointIndex - 1));
        Vector2 nextPoint = new Vector2(path.get(pointIndex + 1));

        double distanceOne = Vector2.distanceFromPoint(point, prevPoint);
        double distanceTwo = Vector2.distanceFromPoint(point, nextPoint);
        double distanceThree = Vector2.distanceFromPoint(nextPoint, prevPoint);

        double productOfSides = distanceOne * distanceTwo * distanceThree;
        double semiPerimeter = (distanceOne + distanceTwo + distanceThree) / 2;
        double triangleArea = Math.sqrt(semiPerimeter * (semiPerimeter - distanceOne) * (semiPerimeter - distanceTwo) * (semiPerimeter - distanceThree));

        double radius = (productOfSides) / (4 * triangleArea);
        double curvature = 1 / radius;

        return curvature;
    }

    private double calculateMaxVelocity(ArrayList<Vector2> path, int point, double pathMaxVel, double k) {
        if (point > 0) {
            double curvature = calculatePathCurvature(path, point);
            return Math.min(pathMaxVel, k / curvature); //k is a constant (generally between 1-5 based on how quickly you want to make the turn)

        }
        return pathMaxVel;
    }

    public double calculateCurrDistance(int point) {
        return robotPath.get(point).getDistance();
    }

    public double getTotalPathDistance() {
        return calculateCurrDistance(robotPath.size() - 1);
    }

    //setter methods that iterate through and set attributes to robotPath

    public void setCurvatures() {
        getStartPoint().setCurvature(0);
        getEndPoint().setCurvature(0);
        for (int i = 1; i < robotPath.size() - 1; i++) {
            robotPath.get(i).setCurvature(calculatePathCurvature(robotPath, i));
        }
    }

    public void setTargetVelocities(double maxVel, double maxAccel, double k) {
        robotPath.get(robotPath.size() - 1).setVelocity(0);
        for (int i = robotPath.size() - 2; i >= 0; i--) {
            double distance = Vector2.distanceFromPoint(robotPath.get(i + 1), robotPath.get(i));
            //System.out.println(robotPath.get(i));
            double maxReachableVel = Math.sqrt(Math.pow(robotPath.get(i + 1).getVelocity(), 2) + (2 * maxAccel * distance));
            robotPath.get(i).setVelocity(Math.min(calculateMaxVelocity(robotPath, i, maxVel, k), maxReachableVel));
        }
    }



    public void setDistances() {
        double distance = 0;
        getStartPoint().setDistance(0);
        for (int i = 1; i < robotPath.size(); i++) {
            distance += Vector2.subtract(robotPath.get(i), robotPath.get(i - 1)).mag();
            robotPath.get(i).setDistance(distance);
        }
    }

    //calculating the curvature necessary for a lookahead arc

    public double calculateCurvatureLookAheadArc(Vector2 currPos, double heading, Vector2 lookahead, double lookaheadDistance) {
        double a = -Math.tan(heading);
        double b = 1;
        double c = (Math.tan(heading) * currPos.x) - currPos.y;
        double x = Math.abs(a * lookahead.x + b * lookahead.y + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double cross = (Math.sin(heading) * (lookahead.x - currPos.x)) - (Math.cos(heading) * (lookahead.y - currPos.y));
        double side = cross > 0 ? 1 : -1;
        double curvature = (2 * x) / (Math.pow(lookaheadDistance, 2));
        return curvature * side;
    }
}