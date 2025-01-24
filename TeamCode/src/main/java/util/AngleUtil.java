package util;

public class AngleUtil {

    //if angle greater 180 flip it around to other way
    // 190 -> -170
    //radians
    public static double clipAngle(double angle) {
        while (Math.abs(angle) > Math.PI) {
            angle -= Math.PI * 2.0 * Math.signum(angle); //signum returns sign: -1 if neg, 1 if pos, 0 if 0
        }
        return angle;
    }
}