package org.firstinspires.ftc.teamcode.util;

public class Vector3 implements Cloneable {
    public double x;
    public double y;
    public double z;
    private double magnitudecache;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public Vector3() {
        x = y = z = 0;
    }

    //transform vector to Pose2d
    public Pose2d toPose() {
        return new Pose2d(x, y, z);
    }

    //add this and another vector together
    public void add(Vector3 a) {
        x += a.x;
        y += a.y;
        z += a.z;
        magnitudecache = 0;
    }

    //subtract this and another vector
    public void subtract(Vector3 a) {
        x -= a.x;
        y -= a.y;
        z -= a.z;
        magnitudecache = 0;
    }

    //get magnitude of this vector
    public double getMag() {
        if (magnitudecache == 0 && ( x != 0 || y != 0 || z != 0)) {
            magnitudecache = Math.sqrt(Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2));
        }
        return magnitudecache;
    }

    //multiply this vector by a scalar
    public void mult(double a) {
        x *=a;
        y *= a;
        z *= a;
        magnitudecache *= a;
    }

    //get unit vector
    public void norm() {
        double mag = getMag();
        x /= mag;
        y /= mag;
        z /= mag;
        magnitudecache = 1;
    }

    //add any two vectors
    public static Vector3 add(Vector3 a, Vector3 b) {
        return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    //subtract any two vectors
    public static Vector3 subtract(Vector3 a, Vector3 b) {
        return new Vector3(a.x-b.x,a.y-b.y,a.z-b.z);
    }

    //mult any vector by a scalar
    public static Vector3 mult(Vector3 a, double b) {
        return new Vector3(a.x * b, a.y * b, a.z * b);
    }

    //returns dot product of two vectors
    public static double dotProduct(Vector3 a, Vector3 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    // projects v onto u
    public static Vector3 project(Vector3 v, Vector3 u) {
        double component = Vector3.dotProduct(v,u) / Vector3.dotProduct(u,u);
        Vector3 temp = new Vector3(u.x, u.y, u.z);
        temp.mult(component);
        return temp;
    }

    public String toString() {
        return String.format("(%f, %f, %f)", x, y, z);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    //returns cloned vector
    @Override
    public Vector3 clone() {
        return new Vector3(x, y, z);
    }

}