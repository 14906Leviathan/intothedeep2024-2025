package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class Auto {
    public Path toBucketPath;
    public Path bucketScorePath;
    private Follower follower;

    public Auto(Follower _follower) {
        follower = _follower;
    }

    public void runPath(Path path) {
        follower.followPath(path);
    }

    public void update() {
        follower.update();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void buildPaths(AutoLocation autoLocation) {
        if(autoLocation == AutoLocation.LEFT_SIMPLE) {
            toBucketPath = new Path(
                    new BezierLine(
                            new Point(7.976, 83.819, Point.CARTESIAN),
                            new Point(37.3, 97.8, Point.CARTESIAN)
                    )
            );

            toBucketPath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120));


            bucketScorePath = new Path(
                    new BezierLine(
                            toBucketPath.getLastControlPoint(),
                            new Point(19.1, 118.4, Point.CARTESIAN)
                    )
            );

            bucketScorePath.setLinearHeadingInterpolation(toBucketPath.getPathEndHeadingConstraint(), Math.toRadians(120));
        }
    }
}
