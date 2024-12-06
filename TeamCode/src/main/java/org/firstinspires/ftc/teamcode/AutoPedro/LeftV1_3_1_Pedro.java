package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipelinePNP;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Left 3+1 V1 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
public class LeftV1_3_1_Pedro extends LinearOpMode {
    private Follower follower;
    private ElapsedTime autoTime = new ElapsedTime();
    private int autoState = 1;
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private int armWaitSleep = 750;
    private int outtakeSleep = 600;
    private int waitForArmIntakeDown = 500;
    private int waitToIntake = 1250;
    private ElapsedTime time = new ElapsedTime();
    private int waitForGrab = 150;
    private int waitForHalt = 500;
    private int intakeSleep = 1000;
    private double s1Xerror = 0;
    private double s1Yerror = 0;
    private double s2Xerror = 0;
    private double s2Yerror = 0;
    private double s3Xerror = 0;
    private double s3Yerror = 0;
    private boolean autoStart = true;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private boolean opModeRunning = true;
    private SampleDetectionPipelinePNP detectionPipe;
    private OpenCvWebcam webcam;
    private AutoManagerPedro autoManager;
    private boolean useLimelight = true;
    private Thread limelightHandler = new Thread(() -> {
        while (opModeIsActive()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                //boo hoo
                break;
            }
        }

        robot.limelight.stop();
    });
    private final boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_3_1_V1;
    private int nextAutoState = 0;
    private Telemetry telemetryA;
    private Pose robotStartPose = new Pose(0, 0, 0);

    public LeftV1_3_1_Pedro() {
    }

    public void runOpMode() {
        follower = new Follower(hardwareMap);
        follower.resetIMU();
        telemetry.update();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);
        arm.poleToucherIn();
        arm.setAutoMode(true);
        arm.setPedroAuto(true);
        arm.resetSlidesPosition();

        intake.setGrabAngle(grabAngle);
        intake.setGrabStyle(grabStyle);
        intake.intake();

        intake.update();
        arm.update();

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update();
            intake.update();
            telemetryA.addData("start x: ", robotStartPose.getX());
            telemetryA.addData("start y: ", robotStartPose.getY());
            telemetryA.addData("limelight fps: ", robot.limelight.getStatus().getFps());
            telemetryA.addData("using LL: ", useLimelight);
            telemetryA.addData("s1 x: ", s1Xerror);
            telemetryA.addData("s1 y: ", s1Yerror);
            telemetryA.addData("s2 x: ", s2Xerror);
            telemetryA.addData("s2 y: ", s2Yerror);
            telemetryA.addData("s3 x: ", s3Xerror);
            telemetryA.addData("s3 y: ", s3Yerror);

            if(robot.limelight.getStatus().getFps() == 0.0 && useLimelight) {
                useLimelight = false;
            }
        }, arm, intake, robot);
        autoManager.buildPaths(autoLocation);
        follower.setPose(autoManager.start_3_1_V1);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);
        intake.intake();

        autoManager.safeSleep(350);

        arm.setSlidesPower(1);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        detectionPipe = new SampleDetectionPipelinePNP();
//
//        webcam.setPipeline(detectionPipe);
//
//        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//
//        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        while (!opModeIsActive()) {
            arm.update();
            intake.update();
            telemetry.addData("limelight connected: ", robot.limelight.isConnected());
            if(robot.limelight.getStatus().getFps() == 0) {
                for(int i = 0; i <= 30; i++) {
                    telemetry.addLine("UNPLUG AND REPLUG THE LIMELIGHT");
                }
            }
            telemetry.addLine("The robot must be started on the third tile");
            telemetry.addLine("to the right from the buckets, the robot must be");
            telemetry.addLine("started on the left line of the tile, and the robot");
            telemetry.addLine("needs to be facing towards the submersible");
            telemetry.addLine();
            telemetry.addData("limelight fps: ", robot.limelight.getStatus().getFps());
            telemetry.addData("slides raw enc:", robot.slidesMotor.getCurrentPosition());
            follower.setStartingPose(autoManager.start_3_1_V1);
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());

//            LLResult result = robot.limelight.getLatestResult();

//            if (result != null) {
//                telemetry.addData("detectX: ", result.getPythonOutput()[0]);
//                telemetry.addData("detectY: ", result.getPythonOutput()[1]);
//            }
            telemetry.update();
        }

        waitForStart();

//        limelightHandler.start();

        if (!robot.limelight.isRunning()) {
            robot.limelight.start();
            autoManager.safeSleep(50);
            robot.limelight.pipelineSwitch(1);
            autoManager.safeSleep(50);
            robot.limelight.pipelineSwitch(3);
            autoManager.safeSleep(50); //jank fix for an issue
        }

        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(autoManager.start_3_1_V1);
        autoManager.safeSleep(100);
        params.AUTO_SCORE = 37;
        params.AUTO_END_HEADING = -90;

        time.reset();
        arm.setArmPower(.6);

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

//        follower.setPose(new Pose(robot.distanceOne.getDistance(DistanceUnit.INCH) + .5, autoManager.start_3_1_V1.getY(), autoManager.start_3_1_V1.getHeading()));
//        follower.updatePose();

//        armHandlerThread.start();
//        autoManager.runPath(autoManager.toBucketPath, false);

//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setBucket(2);
//        autoManager.safeSleep(750);

//        bucketScore(1);
        robotStartPose = follower.getPose();

        specimenScore(1);
        intake(1);
        bucketScore(1);
        intake(2);
        bucketScore(2);
        intake(3);
        bucketScore(3);
//        park();

        params.AUTO_END_HEADING = Math.toDegrees(follower.getTotalHeading());

        telemetryA.addData("time: ", time.time(TimeUnit.SECONDS));
        telemetryA.update();
        autoManager.safeSleep(10000);
    }

    public void specimenScore(int sampleNum) {
        autoManager.setSpeed(1);
        if (sampleNum == 1) {
            currentMode = TeleopMode.SPECIMEN_SCORE;
            autoManager.runPath(autoManager.specScore1, true);
        }
        autoManager.setSpeed(1);
        autoManager.safeSleep(350);
//        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        arm.setArmPositionSpecimen(75); //params.ARM_SCORE_SPECIMEN
        arm.setSlidesPositionSpecimen(6); //params.SLIDES_SCORE_SPECIMEN
        autoManager.safeSleep(500);
        arm.setArmPositionSpecimen(95); //params.ARM_SCORE_SPECIMEN
        arm.setSlidesPositionSpecimen(0); //params.SLIDES_SCORE_SPECIMEN
        autoManager.safeSleep(350);
//        autoManager.safeSleep(30_000);
//        autoManager.runPath(new PathBuilder()
//                .addPath(new Path(
//                        new BezierLine(
//                                autoManager.specScore1.build().getPath(autoManager.specScore1.build().size() - 1).getLastControlPoint(),
//                                new Point(30.0, autoManager.specScore1.build().getPath(autoManager.specScore1.build().size() - 1).getLastControlPoint().getY(), Point.CARTESIAN)
//                        )
//                )), false);
//        arm.setSlidesPositionSpecimen(params.SLIDES_ENSURE_SCORE_SPECIMEN);
//        autoManager.safeSleep(350);

        intake.outtake();
        intake.update();
        autoManager.safeSleep(200);
//        autoManager.safeSleep(30000);

        autoManager.runPath(autoManager.specBackup);
    }

    public void park() {
        autoManager.setSpeed(1);
        params.TELEOP_START_MODE = TeleopMode.TOUCH_POLE_AUTO;
        autoManager.runPath(autoManager.park, false);
//        arm.update();
//        autoManager.safeSleep(750);
//        arm.setParkArmUp(false);
//        autoManager.safeSleep(10000);
    }

    public void intake(int sampleNum) {
        arm.setIntakePosition(params.PEDRO_AUTO_INTAKE_Y1_POS);
        arm.intakeUpMode();
        if (sampleNum != 3) {
//            arm.intakeDownMode();
            arm.intakeUpMode();
        } else {
            arm.intakeUpMode();
        }

        Pose error = new Pose();

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
        if (sampleNum == 1) {
            autoManager.setSpeed(.7);
            autoManager.runPath(autoManager.intakeYellow1, true);
            autoManager.safeSleep(200);
//            autoManager.waitForArmAndSlides(300);
            error = autoManager.homeToSample(Math.toRadians(0));
        } else if (sampleNum == 2) {
            autoManager.setSpeed(.7);
            autoManager.runPath(autoManager.intakeYellow2, true);
            autoManager.safeSleep(700);
            autoManager.waitForArmAndSlides(500);
            error = autoManager.homeToSample(Math.toRadians(0));
        } else if (sampleNum == 3) {
            autoManager.setSpeed(.8);
            arm.setAutoLastSample(true);
            intake.setGrabAngle(GrabAngle.CUSTOM);
            intake.setCustomGrabAngle(135);
            arm.intakeUpMode();
            autoManager.runPath(autoManager.intakeYellow3, true);
            arm.intakeUpMode();
            autoManager.safeSleep(200);
            autoManager.waitForArmAndSlides(500);
            error = autoManager.homeToSample(Math.toRadians(0));
        }

        autoManager.setSpeed(1);

        if(sampleNum != 3) {
            arm.intakeDownMode();

            autoManager.holdPoint(new Pose(
                    follower.getPose().getX() + 4.5,
                    follower.getPose().getY(),
                    follower.getTotalHeading()
            ));
        } else {
            autoManager.holdPoint(new Pose(
                    follower.getPose().getX() + 2.75,
                    follower.getPose().getY() + 0,
                    follower.getTotalHeading()
            ));

            arm.intakeDownMode();
        }
        autoManager.safeSleep(850);
//        autoManager.waitForArmAndSlides(1000);

        intake.intake();
        autoManager.safeSleep(250);

        if (sampleNum == 1) {
            s1Xerror = error.getX();
            s1Yerror = error.getY();
        } else if (sampleNum == 2) {
            s2Xerror = error.getX();
            s2Yerror = error.getY();
        } else if (sampleNum == 3) {
            s3Xerror = follower.getPose().getX();
            s3Yerror = follower.getPose().getY();
        }

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        currentMode = TeleopMode.BUCKET_SCORE;
        arm.setTeleopMode(currentMode);
//        arm.setAnimationType(AnimationType.FAST);
        arm.setBucket(2);
        arm.update();
        autoManager.safeSleep(1250);
    }

    public void bucketScore(int sampleNum) {
        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
        intake.update();

        if (arm.getTeleopMode() != TeleopMode.BUCKET_SCORE) {
            currentMode = TeleopMode.BUCKET_SCORE;
            arm.setTeleopMode(currentMode);
            arm.setAnimationType(AnimationType.NONE);
            arm.setBucket(2);
            if (sampleNum == 1) {
                autoManager.safeSleep(1000);
            } else {
                autoManager.safeSleep(100);
            }
        }

        autoManager.setSpeed(.8);
        if (sampleNum == 1) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake1, true);
        } else if (sampleNum == 2) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake2, true);
        } else if (sampleNum == 3) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake3, true);
        }

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        autoManager.safeSleep(200);
        arm.setArmTipBucketScore(true);
        autoManager.safeSleep(350);
        intake.outtake();
        autoManager.safeSleep(450);
        arm.setArmTipBucketScore(false);
        autoManager.safeSleep(400);

        if (sampleNum != 4) {
//            autoManager.setSpeed(.6);
//            autoManager.runPath(autoManager.backupPath, false);
//            currentMode = TeleopMode.INTAKE;
//            arm.setTeleopMode(currentMode);
//            arm.setIntakePosition(params.PEDRO_AUTO_INTAKE_Y1_POS);
//            arm.intakeUpMode();
//            arm.update();
//            autoManager.safeSleep(250);
//            autoManager.setSpeed(1);
        }
    }
}
