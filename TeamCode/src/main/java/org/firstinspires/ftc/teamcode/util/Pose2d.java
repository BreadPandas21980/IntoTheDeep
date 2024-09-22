package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

//Pose2d is a point with x, y, and heading
public class Pose2d implements Cloneable {

    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y){
        this(x,y,0);
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    //adds pose to this pose
    public void add(Pose2d p1){
        this.x += p1.x;
        this.y += p1.y;
        this.heading += p1.heading;
    }
    //subtracts pose to this pose
    public void subtract(Pose2d p1){
        this.x -= p1.x;
        this.y -= p1.y;
        this.heading -= p1.heading;
    }

    //checks if pose NaN
    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading(){
        return heading;
    }

    public double getDistanceFromPoint(Pose2d newPoint) { // distance equation
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
    }

    //difference from our x to new point x
    public double getErrorInX(Pose2d newPoint) { // distance equation
        return Math.abs(x - newPoint.x);
    }
    //same with y
    public double getErrorInY(Pose2d newPoint) { // distance equation
        return Math.abs(y - newPoint.y);
    }

    //flip the angle if its bigger than PI the other way
    // 190 -> -170
    public void clipAngle() {
        heading = AngleUtil.clipAngle(heading);
    }

    //convert Pose2d to Vector3
    public Vector3 toVec3() {
        return new Vector3(x, y, heading);
    }

    @NonNull
    @Override
    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }
}