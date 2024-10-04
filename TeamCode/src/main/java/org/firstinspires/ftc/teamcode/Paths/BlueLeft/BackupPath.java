package org.firstinspires.ftc.teamcode.Paths.BlueLeft;

import org.firstinspires.ftc.teamcode.Abstracts.AutoPath;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class BackupPath extends AutoPath {
    BucketScorePath lastPath = new BucketScorePath();

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
          new Point(lastPath.getEndX(), lastPath.getEndY(), Point.CARTESIAN),
          new Point(24.943, 118.332, Point.CARTESIAN)
        )
      ).setLinearHeadingInterpolation(Math.toRadians(lastPath.getEndHeading()), Math.toRadians(140));

        return builder;
    }
}
