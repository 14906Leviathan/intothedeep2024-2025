package org.firstinspires.ftc.teamcode.Paths.BlueLeft;

import org.firstinspires.ftc.teamcode.Abstracts.AutoPath;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class ToBucketPath extends AutoPath {
    public double getEndX() {
        return 14.502;
    }

    public double getEndY() {
        return 130.079;
    }

    public double getEndHeading() {
        return 140;
    }

    public PathBuilder getPath() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.976, 83.819, Point.CARTESIAN),
                                new Point(37.3, 97.8, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120));

        return builder;
    }
}