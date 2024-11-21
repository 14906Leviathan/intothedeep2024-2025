package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Path test", group = "Autonomous Pathing Tuning")
public class PathTest extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private PathBuilder forwards;
    private Path backwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(9, 106, Math.toRadians(0)));
        follower.setMaxPower(.5);

        forwards = new PathBuilder().addPath(new Path(
                new BezierLine(
                        new Point(9, 106, Point.CARTESIAN),
                        new Point(25, 97.8, Point.CARTESIAN)
                )
        ));
        forwards.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45));
        forwards.addParametricCallback(.99, () -> {
            follower.setMaxPower(1);
        });
        backwards = new Path(
                new BezierLine(
                        new Point(25, 97.8, Point.CARTESIAN),
                        new Point(9, 106, Point.CARTESIAN)
                )
        );
        backwards.setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0));

        follower.followPath(forwards.build(), true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
//            requestOpModeStop();
            if (forward) {
                forward = false;
//                follower.followPath(backwards);
            } else {
                forward = true;
//                follower.followPath(forwards, true);
            }
        }

//        telemetryA.addData("going forward", forward);
        telemetryA.addData("x: ", follower.getPose().getX());
        telemetryA.addData("y: ", follower.getPose().getY());
        follower.telemetryDebug(telemetryA);
    }
}

