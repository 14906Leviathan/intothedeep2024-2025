package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class Auto {
    public Path toBucketPath;
    public Path backupPath;
    public Path bucketScorePath;
    public Path park;
    private Follower follower;
    private boolean pathingDisabled = false;

    public Auto(Follower _follower) {
        follower = _follower;
    }

    public void runPath(Path path) {
        if(!pathingDisabled) follower.followPath(path);
    }

    public void update() {
        follower.update();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void disablePathing(boolean disabled) {
        pathingDisabled = disabled;
    }

    public void holdCurrentPoint() {
        BezierPoint currentPoint = new BezierPoint(follower.getCurrentPath().getLastControlPoint());
        follower.holdPoint(currentPoint, follower.getCurrentPath().getPathEndHeadingConstraint());
    }

    public void setSpeed(double speed) {
        follower.setMaxPower(speed);
    }

    public void buildPaths(AutoLocation autoLocation) {
        if(autoLocation == AutoLocation.LEFT_SIMPLE) {
            toBucketPath = new Path(
                    new BezierLine(
                            new Point(7.976, 83.819, Point.CARTESIAN),
                            new Point(25, 97.8, Point.CARTESIAN)
                    )
            );

            toBucketPath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120));


            bucketScorePath = new Path(
                    new BezierLine(
                            toBucketPath.getLastControlPoint(),
                            new Point(21, 108, Point.CARTESIAN)
                    )
            );

            bucketScorePath.setConstantHeadingInterpolation(Math.toRadians(140));
//            bucketScorePath.setLinearHeadingInterpolation(toBucketPath.getPathEndHeadingConstraint(), Math.toRadians(150));

            backupPath = new Path(
                    new BezierLine(
                            bucketScorePath.getLastControlPoint(),
                            new Point(25, 95, Point.CARTESIAN)
                    )
            );

            backupPath.setConstantHeadingInterpolation(Math.toRadians(140));

            park = new Path(
                    new BezierCurve(
                            backupPath.getLastControlPoint(),
                            new Point(68, 80, Point.CARTESIAN),
                            new Point(64, 80, Point.CARTESIAN)
                    )
            );

            park.setConstantHeadingInterpolation(Math.toRadians(90));
        }
    }
}
