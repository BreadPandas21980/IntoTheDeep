package org.firstinspires.ftc.teamcode.util;

public class Vector2 {
    public double x;
    public double y;
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
}
