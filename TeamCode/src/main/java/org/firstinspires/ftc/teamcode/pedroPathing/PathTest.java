package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Path Test", group = "Autonomous Pathing Tuning")
public class PathTest extends LinearOpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;
    private ArmSubsystem arm;
    private HWProfile robot;
    private boolean pathDone = false;
    private ElapsedTime elapsedTime = new ElapsedTime();

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        follower.resetIMU();

//        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
//        backwards.setConstantHeadingInterpolation(0);

        follower.setPose(new Pose(9, 83.5, Math.toRadians(0)));

        forwards = new Path(
                new BezierLine(
                        new Point(9, 83.5, Point.CARTESIAN),
                        new Point(23, 102, Point.CARTESIAN)
                )
        );
        forwards.setConstantHeadingInterpolation(Math.toRadians(0));

//        follower.followPath(forwards);
        robot = new HWProfile();
        robot.init(hardwareMap, false);
        arm = new ArmSubsystem(robot, this, new Params());

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            arm.setTeleopMode(TeleopMode.IDLE);
            arm.update();

            follower.update();
            if (!follower.isBusy() && !pathDone) {
                follower.followPath(forwards);
            } else {
                pathDone = true;
            }

            if(pathDone && elapsedTime.time(TimeUnit.SECONDS) > 10) {
                follower.holdPoint(new BezierPoint(new Point(23, 102, Point.CARTESIAN)), Math.toRadians(0));
            }

            telemetryA.addData("going forward", forward);
            follower.telemetryDebug(telemetryA);

            arm.update();
        }
    }
}
