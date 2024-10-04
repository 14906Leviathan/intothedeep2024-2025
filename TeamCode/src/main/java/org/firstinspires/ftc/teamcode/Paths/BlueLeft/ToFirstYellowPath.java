package org.firstinspires.ftc.teamcode.Paths.BlueLeft;

import org.firstinspires.ftc.teamcode.Abstracts.AutoPath;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.*;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class ToFirstYellowPath extends AutoPath {
    private BucketScorePath lastPath;

    public double getEndX() {
        return 24.798;
    }

    public double getEndY() {
        return 120.943;
    }

    public double getEndHeading() {
        return 0;
    }

    public PathBuilder getPath() {
        PathBuilder builder = new PathBuilder();

    builder
      .addPath(
        // Line 1
        new BezierCurve(
          new Point(lastPath.getEndX(), lastPath.getEndY(), Point.CARTESIAN),
          new Point(13.921, 120.363, Point.CARTESIAN),
          new Point(24.798, 120.943, Point.CARTESIAN)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(lastPath.getEndHeading()), Math.toRadians(0));

        return builder;
    }
}
