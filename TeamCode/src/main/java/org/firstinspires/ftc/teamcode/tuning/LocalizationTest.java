package org.firstinspires.ftc.teamcode.tuning;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import org.firstinspires.ftc.teamcode.Misc.AprilTagDrive;
import org.firstinspires.ftc.teamcode.Misc.Drawing;
import org.firstinspires.ftc.teamcode.AutoRoadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Misc.TankDrive;

@TeleOp(name = "localization test")
public class LocalizationTest extends LinearOpMode {
    private HWProfile robot;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new HWProfile();
        robot.init(hardwareMap, false, false);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
            String webcamName = "Webcam 1";
            VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(aprilTag)
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .setCameraResolution(new Size(640, 480))
//                    .setLiveViewContainerId(0)
                    .enableLiveView(true)
                    .build();

            AprilTagProcessor aprilTag2 = AprilTagProcessor.easyCreateWithDefaults();
            String webcamName2 = "Webcam 2";
            VisionPortal visionPortal2 = new VisionPortal.Builder()
                    .addProcessor(aprilTag2)
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .setCameraResolution(new Size(640, 480))
//                    .setLiveViewContainerId(0)
                    .enableLiveView(false)
                    .build();

            MecanumDrive drive = new AprilTagDrive(hardwareMap, new Pose2d(0, 0, 0), aprilTag);

            waitForStart();

            while (opModeIsActive()) {
//                drive.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -gamepad1.left_stick_y,
//                                -gamepad1.left_stick_x
//                        ),
//                        -gamepad1.right_stick_x
//                ));

                drive.updatePoseEstimate();

//                telemetry.addData("distance one: ", robot.distanceOne.getDistance(DistanceUnit.INCH));
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
