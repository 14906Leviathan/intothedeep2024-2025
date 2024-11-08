package org.firstinspires.ftc.teamcode.AutoRoadrunner;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SampleDetectionPipelinePNP;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Config
public class AutoManager {
    public TrajectoryActionBuilder toBucketPath;
    public Vector2d strafeToBucket;
    public Double strafeToBucketHeading;
    public TrajectoryActionBuilder backupPath;
    public TrajectoryActionBuilder bucketScorePath;
    public TrajectoryActionBuilder park;
    public Vector2d intakeYellow1Path;
    public Double intakeYellow1PathHeading;
    public Vector2d intakeYellow2Path;
    public Double intakeYellow2PathHeading;
    public Vector2d intakeYellow2StrafeTo;
    public Double intakeYellow2StrafeToHeading;
    public Vector2d intakeYellow3Path;
    public Double intakeYellow3PathHeading;
    public Vector2d intakeYellow1StrafeTo;
    public Double intakeYellow1StrafeToHeading;
    public Vector2d intakeYellow3StrafeTo;
    public Double intakeYellow3StrafeToHeading;
    public TrajectoryActionBuilder intakeYellow3;
    private MecanumDrive drive;
    private Pose2d currentPose;
    private int XInvert = 1;
    private int YInvert = 1;
    public static double driveKp = .02;
    public static double driveKi = .2;
    public static double driveKd = .01;
    private PIDController driveHomePid = new PIDController(driveKp, driveKi, driveKd);
    public double XOffset = 61.6;
    public double YOffset = 1.3;
    private LinearOpMode opMode;
    private boolean pathingDisabled = false;
    private Pose2d leftParkLocation = new Pose2d(64, 89, Math.toRadians(0));
    private Pose2d leftScoreLocation = new Pose2d(19, 109, Math.toRadians(140));
    public Pose2d leftIntakeTwoStartPosition = new Pose2d(0, 0, Math.toRadians(0));
    public Pose2d leftIntakeThreeStartPosition = new Pose2d(.5, -.7, Math.toRadians(90));

    public AutoManager(LinearOpMode newOpMode) {
        opMode = newOpMode;
        driveHomePid.setTolerance(.25);
    }

    public AprilTagProcessor getATAGProcessor() {
        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        String webcamName = "Webcam 1";
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(opMode.hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480))
//                    .setLiveViewContainerId(0)
                .enableLiveView(true)
                .build();

        return aprilTag;
    }
//
//    public void homeToSample(SampleDetectionPipelinePNP pipeline) {
//        ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> samples = pipeline.getDetectedStones();
//
//        if (!samples.isEmpty()) {
//            double sampleX = samples.get(0).tvec.get(0, 0)[0] * this.drive.PARAMS.senseInMultX;
//            double sampleY = samples.get(0).tvec.get(1, 0)[0] * this.drive.PARAMS.senseInMultY;
//            drive.extraCorrection = true;
//            runPath(drive.actionBuilder(currentPose)
//                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x - (3 - sampleX) * 0.254065041, drive.pose.position.y), Math.toRadians(90)));
//        }
//    }

    public void setDrive(MecanumDrive _drive) {
        this.drive = _drive;
    }

    public void updatePose(Pose2d newPose) {
        currentPose = newPose;
    }

    public void runPath(TrajectoryActionBuilder path) {
        if (!pathingDisabled) Actions.runBlocking(path.build());
    }

    public void disablePathing(boolean disabled) {
        pathingDisabled = disabled;
    }

    public void buildPaths(AutoLocation autoLocation) {
        if (autoLocation == AutoLocation.LEFT_SIMPLE) {
            toBucketPath = drive.actionBuilder(leftIntakeTwoStartPosition)
                    .strafeToLinearHeading(new Vector2d(15, 9), Math.toRadians(140));

            bucketScorePath = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(new Vector2d(2, 21), Math.toRadians(140));

            backupPath = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(new Vector2d(15, 11), Math.toRadians(0));

        } else if (autoLocation == AutoLocation.LEFT_SCORE_TWO_GOLD) {
            toBucketPath = drive.actionBuilder(leftIntakeTwoStartPosition)
                    .strafeToLinearHeading(new Vector2d(15, 9), Math.toRadians(140));

            bucketScorePath = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(new Vector2d(3.5, 18.5), Math.toRadians(140));

            backupPath = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(new Vector2d(15, 11), Math.toRadians(0));

            intakeYellow1Path = new Vector2d(38.5, -5.35);
            intakeYellow1PathHeading = Math.toRadians(0);

            intakeYellow2Path = new Vector2d(16.4, 18);
            intakeYellow2PathHeading = Math.toRadians(0);
        } else if (autoLocation == AutoLocation.LEFT_SCORE_THREE_GOLD) {
            toBucketPath = drive.actionBuilder(leftIntakeTwoStartPosition)
                    .strafeToLinearHeading(new Vector2d(15, 9), Math.toRadians(140));

            bucketScorePath = drive.actionBuilder(currentPose)
                    .strafeToLinearHeading(new Vector2d(3.5, 20.5), Math.toRadians(140));

            backupPath = drive.actionBuilder(currentPose);

//            intakeYellow1Path = drive.actionBuilder(currentPose)
//                    .strafeToLinearHeading(new Vector2d(36, -5), Math.toRadians(90));
            intakeYellow1Path = new Vector2d(38.5, -5.35);
            intakeYellow1PathHeading = Math.toRadians(90);

            intakeYellow1StrafeTo = new Vector2d(33, intakeYellow1Path.y);
            intakeYellow1StrafeToHeading = intakeYellow1PathHeading;

            intakeYellow2Path = new Vector2d(38.5, 4.4);
            intakeYellow2PathHeading = Math.toRadians(90);
            intakeYellow2StrafeTo = new Vector2d(34, 4.4);
            intakeYellow2StrafeToHeading = Math.toRadians(90);

            intakeYellow3Path = new Vector2d(38, 16.5);
            intakeYellow3PathHeading = Math.toRadians(90);
            intakeYellow3StrafeTo = new Vector2d(37, 9);
            intakeYellow3StrafeToHeading = Math.toRadians(90);
        }
    }
}