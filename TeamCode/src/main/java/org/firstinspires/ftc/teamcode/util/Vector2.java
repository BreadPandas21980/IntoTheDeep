package org.firstinspires.ftc.teamcode.util;

public class Vector2 {
    public double x;
    public double y;
    public double velocity = 0;
    public double curvature = 0;
    public double targetVelocity = 0;
    public double lookAheadRadius;
    private double magnitudecache;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(){
        x = y = 0;
    }

    //adds two vectors together
    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    //returns the magnitude of the vector
    public double mag() {
        if (magnitudecache == 0 && (x != 0 || y!= 0)) {
            magnitudecache = Math.sqrt( Math.pow(x, 2) + Math.pow(y, 2));
        }

        return magnitudecache;
    }

    //multiplies the vector by a number
    public void mult(double a) {
        x *= a;
        y *= a;
        magnitudecache *= a;
    }

    //returns the dot product of 2 vectors
    // ???
    public static double dotProduct(Vector2 a, Vector2 b) {
        return (a.x * b.x + a.y * b.y);
    }

    //change to unit vector
    public void norm() {
        double mag = mag();

        if(mag == 0) {
            return;
        }

        x /= mag;
        y /= mag;
        magnitudecache = 1;
    }

    //adds a vector to this vector
    public void add(Vector2 a) {
        x += a.x;
        y += a.y;
        magnitudecache = 0;
    }

    //vector to string format
    public String toString() {
        return String.format("(%f, %f)", x, y);
    }

    //rotate this vector using rotation matrix
    public void rotate(double angle) {
        x = x * Math.cos(angle) + y * Math.sin(angle);
        y = x * -Math.sin(angle) + y * Math.cos(angle);
    }

    //rotate any vector using rotation matrix
    public static Vector2 rotate(Vector2 vector, double angle) {
        double x = vector.x;
        double y = vector.y;
        x = x * Math.cos(angle) + y * Math.sin(angle);
        y = x * -Math.sin(angle) + y * Math.cos(angle);
        return new Vector2(x, y);
    }

    //rotate this vector around a point
    //https://stackoverflow.com/questions/620745/c-rotating-a-vector-around-a-certain-point
    public void rotateAround(double angle, double x, double y) {
        this.x -= x;
        this.y -= y;
        rotate(angle);
        this.x += x;
        this.y += y;
    }

    //rotate any vector around a point
    public static Vector2 rotateAround(Vector2 vector, double angle, double x, double y) {
        double vx = vector.x - x;
        double vy = vector.y - y;
        Vector2 temp = Vector2.rotate(new Vector2(vx, vy), angle);
        return new Vector2(temp.x + x, temp.y + y);
    }
    public double getDistanceFromPoint(Vector2 newPoint) { // distance equation
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
    }

    public static double[][] oneDtoTwoD(Vector2[] path) {
        double[][] pathCopy = new double[path.length][2];
        for (int i = 0; i < path.length; i++) {
            for(int j = 0; j < 2; j++) {
                pathCopy[i][0] = path[i].x;
                pathCopy[i][1] = path[i].y;
            }
        }
        return pathCopy;
    }

    public static Vector2[] twoDtoOneD(double[][] path) {
        Vector2[] pathCopy = new Vector2[path.length];
        for (int i = 0; i < path.length; i++) {
            pathCopy[i] = new Vector2(path[i][0], path[i][1]);
        }
        return pathCopy;
    }

    //Finding the distance along the path to a point is as simple as keeping a
    // running sum of the distances between points:
    public static double distanceAlongPath(Vector2[] path, Vector2 point) {
        int index = -1;
        for(int i = 0; i < path.length; i++) {
            if(point.equals(path[i])) {
                index = i;
            }
        }
        if(index == 0) {
            return point.mag();
        } else {
            return path[index - 1].mag() + point.getDistanceFromPoint(path[index - 1]);
        }
    }


    //https://www.desmos.com/calculator/obvgxhsz40
    //find curvature by getting radius of cicle that intersects point and 2 points around it
    public static void curvatureOfPath(Vector2[] path) {
        /*
        double[] curvature = new double[path.length];
        curvature[0] = 0;
        curvature[path.length - 1] = 0;

         */
        path[0].curvature = 0;
        path[path.length - 1].curvature = 0;

        for(int i = 1; i < path.length - 1; i++) {
            double x1 = path[i-1].x + 0.0001; //prevent divide by zero if x1=x2
            double y1 = path[i-1].y;
            double x2 = path[i].x;
            double y2 = path[i].y;
            double x3 = path[i+1].x;
            double y3 = path[i+1].y;

            double k1 = 0.5 * (x1*x1 + y1*y1 - x2*x2 - y2*y2) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);
            double b = 0.5 * (x2*x2 - 2*x2*k1 +y2*y2 - x3*x3 + 2*x3*k1 - y3*y3) / (x3*k2 - y3 + y2 - x2*k2);
            double a = k1 - k2*b;
            double radius = Math.sqrt( Math.pow((x1-a), 2) + Math.pow((y1-b), 2) );
            double curvatureOfPoint = 1 / radius;

            //If the answer is NaN, this means the radius is ∞ ,
            // the curvature is 0, and the path is a straight line
            if(Double.isNaN(curvatureOfPoint)) {
                curvatureOfPoint = 0;
            }

            path[i].curvature = curvatureOfPoint;
        }
    }

    public static void findPointVelocity(Vector2[] path) {

        double maxPathVelocity = 60; //in/s
        double k = 3; //k is a constant around 1-5, based on how slow you want the robot to go around turns.
        for(int i = 0; i < path.length; i++) {
            path[i].velocity = Math.min(maxPathVelocity, k / path[i].curvature);
        }
    }

    public static void findTargetVelocityAtPoint(Vector2[] path) {

        //target velocity is minimum of point's current velocity and largest velocity reachable when starting at last point
        //vf is maximum reachable velocity at point
        //vi is velocity at last point
        //a is max accel
        //d is dist btwn points
        //vf^2 = vi^2 + 2aDx
        //vf = sqrt(vi^2 + 2ad)
        double a = 60; //in/s
        findPointVelocity(path);
        path[path.length - 1].velocity = 0;
        for(int i = path.length - 2; i > -1; i--) {
            double distance = path[i + 1].getDistanceFromPoint(path[i]);
            path[i].targetVelocity = Math.min(
                    path[i].velocity,
                    Math.sqrt(  Math.pow(path[i + 1].targetVelocity, 2) + 2 * a * distance  )
            );
        }
    }


}
