package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.types.PathType;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@TeleOp(name = "Pure Pursuit Test", group = "Test")
@Config
public class PurePursuitTest extends LinearOpMode {
    PureTwoWheelOdo odometry;
    HWProfile robot = new HWProfile();
    private MecanumDrive mecDrive;
    public static double followRadius = 1;
    private static double movementSpeed = 1;
    private static double radiusSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true,true);

        odometry = new PureTwoWheelOdo(new Pose2d(new Translation2d(0,0), new Rotation2d(Math.toRadians(0))), hardwareMap, robot.imu);

        mecDrive = new MecanumDrive(robot.motorLF, robot.motorRF, robot.motorLR, robot.motorRR);
        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
                Pose2d pose = odometry.getPose();

//                telemetry.addData("x: ", pose.getX());
//                telemetry.addData("y: ", pose.getY());
//                telemetry.addData("heading: ", Math.toDegrees(pose.getHeading()));
//                telemetry.update();
            }
        }).start();

        while (opModeIsActive()) {
            odometry.updatePose();
            Pose2d pose = odometry.getPose();

            if(gamepad1.a) {
                Waypoint startWaypoint = new StartWaypoint(0, 0);
                Waypoint p1 = new PointTurnWaypoint(
                        0, 15, movementSpeed,
                        1, followRadius,
                        2, Math.toRadians(2)
                );
                Waypoint endPoint = new EndWaypoint(
                        15, 15, Math.toRadians(0), movementSpeed,
                        1, followRadius,
                2, Math.toRadians(2)
                );

                Path path = new Path(startWaypoint, p1, endPoint);
                path.setPathType(PathType.HEADING_CONTROLLED);
                path.init();
                while (!path.isFinished() && opModeIsActive()) {
                    if (path.timedOut())
                        throw new InterruptedException("Timed out");

                    odometry.updatePose();

                    Pose2d robotPos = odometry.getPose();

                    // return the motor speeds
                    double speeds[] = path.loop(robotPos.getX(), robotPos.getY(),
                            robotPos.getHeading());

                    mecDrive.driveRobotCentric(speeds[0], speeds[1], -speeds[2]);

                    telemetry.addData("speeds 2: ", speeds[2]);
                    telemetry.update();
                }
                mecDrive.stop();
            } else if(gamepad1.b) {
                Waypoint startWaypoint = new StartWaypoint(15, 15);
                Waypoint endPoint = new EndWaypoint(
                        0, 0, Math.toRadians(0), 1,
                        1, followRadius,
                        2, Math.toRadians(2)
                );

                Path path = new Path(startWaypoint, endPoint);
                path.init();
                path.followPath(mecDrive, odometry);
            }
        }
    }
}