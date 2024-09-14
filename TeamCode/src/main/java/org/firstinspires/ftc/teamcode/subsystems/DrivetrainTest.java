package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Vector2;

public class DrivetrainTest {

    public Vector2 lineCircleIntersection(Pose2d robotPose, Pose2d lineStart, Pose2d lineEnd, double lookAheadRadius) {

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
            }
            if (t2 >= 0 && t2 <= 1) {

            } else {
                return null;
            }
        }
    }


}
