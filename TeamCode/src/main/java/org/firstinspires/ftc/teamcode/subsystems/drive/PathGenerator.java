package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.ArrayList;
import java.util.Arrays;

public class PathGenerator {
    private double spacing;
    private double a = 0, b = 0, tolerance = 0;
    private ArrayList<Vector2> points = new ArrayList<>();
    private double maxVel, maxAccel, maxVelk;
    private boolean forward;

    public PathGenerator(double spacing, boolean forward) {
        this.spacing = spacing;
        this.forward = forward;
    }

    public void setSmoothingParameters(double a, double b, double tolerance) {
        this.a = a;
        this.b = b;
        this.tolerance = tolerance;
    }

    public void setVelocities(double maxVel, double maxAccel, double maxVelk) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxVelk = maxVelk;
    }

    public void addPoint(Vector2 point) {
        points.add(point);
    }

    public void addPoints(Vector2... points) {
        this.points.addAll(Arrays.asList(points));
    }

    public Path generatePath() {
        Path path = new Path(spacing, forward);
        for (int i = 0; i < points.size() - 1; ++i)
            path.addSegment(points.get(i), points.get(i + 1));
        path.addLastPoint();
        if (tolerance != 0)
            path.smooth(a, b, tolerance);
        path.initializePath(maxVel, maxAccel, maxVelk);
        return path;
    }
}