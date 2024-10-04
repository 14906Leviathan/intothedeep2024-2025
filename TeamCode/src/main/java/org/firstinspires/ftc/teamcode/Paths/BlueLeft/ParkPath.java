package org.firstinspires.ftc.teamcode.Paths.BlueLeft;

import org.firstinspires.ftc.teamcode.Abstracts.AutoPath;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class ParkPath extends AutoPath {
    BucketScorePath lastPath = new BucketScorePath();

    public double getEndX() {
        return 9.861;
    }

    public double getEndY() {
        return 35.239;
    }

    public double getEndHeading() {
        return -90;
    }

    public PathBuilder getPath() {
        PathBuilder builder = new PathBuilder();

    builder
      .addPath(
        // Line 1
        new BezierLine(
          new Point(lastPath.getEndX(), lastPath.getEndY(), Point.CARTESIAN),
          new Point(9.861, 35.239, Point.CARTESIAN)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(lastPath.getEndHeading()), Math.toRadians(-90));

        return builder;
    }
}
