package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Left 4+0 V1 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
public class LeftV1_4_0_Pedro extends LinearOpMode {
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
    private AutoManagerPedro autoManager;
    private Thread armHandlerThread = new Thread(() -> {
        while (opModeIsActive()) {
//            arm.update();
//            intake.update();
//            autoManager.update();

            telemetry.addData("X:", follower.getPose().getX());
            telemetry.addData("Y:", follower.getPose().getY());
            telemetry.addData("heading:", Math.toDegrees(follower.getTotalHeading()));
            telemetry.addData("autoManager state:", autoState);
            telemetry.update();
        }
    });
    private final boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_4_0_V1;
    private int nextAutoState = 0;
    private final String autoName = "Left Intake Autonomous: PEDRO PATHING";
    private Telemetry telemetryA;

    public String getAutoName() {
        return autoName;
    }

    public LeftV1_4_0_Pedro() {
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
        arm.resetSlidesPosition();

        intake.setGrabAngle(grabAngle);
        intake.setGrabStyle(grabStyle);
        intake.intake();

        intake.update();
        arm.update();

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update();
            intake.update();
        }, arm, intake, robot);
        autoManager.buildPaths(autoLocation);
        follower.setPose(autoManager.start_4_0_V1);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);
        intake.intake();

        autoManager.safeSleep(350);

        arm.setSlidesPower(1);

        while (!opModeIsActive()) {
            arm.update();
            intake.update();
            telemetry.addLine("The robot must be started on the second tile");
            telemetry.addLine("to the right from the buckets, the robot must be");
            telemetry.addLine("started on the left line of the tile, and the robot");
            telemetry.addLine("needs to be facing towards the bucket");
            telemetry.addLine();
            telemetry.addData("slides raw enc:", robot.slidesMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        time.reset();
        arm.setArmPower(.7);

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(autoManager.start_4_0_V1);
        autoManager.safeSleep(100);
        params.AUTO_SCORE = 35;
        params.AUTO_END_HEADING = -90;
//        autoManager.runPath(autoManager.toBucketPath, false);

//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setBucket(2);
//        autoManager.safeSleep(750);

        bucketScore(1);
        intake(1);
        bucketScore(2);
        intake(2);
        bucketScore(3);
        intake(3);
        bucketScore(4);
        park();

        params.AUTO_END_HEADING = Math.toDegrees(follower.getTotalHeading());
        params.TELEOP_START_MODE = TeleopMode.INTAKE;
        params.AUTO_SCORE = 32;

        telemetryA.addData("time: ", time.time(TimeUnit.SECONDS));
        telemetryA.update();
        telemetryA.addData("s1 x: ", s1Xerror);
        telemetryA.addData("s1 y: ", s1Yerror);
        telemetryA.addData("s2 x: ", s2Xerror);
        telemetryA.addData("s2 y: ", s2Yerror);
        telemetryA.addData("s3 x: ", s3Xerror);
        telemetryA.addData("s3 y: ", s3Yerror);
        autoManager.safeSleep(10000);
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
//        arm.intakeUpMode();
        if(sampleNum != 3) {
            arm.intakeDownMode();
        } else {
            arm.intakeUpMode();
        }

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
        if (sampleNum == 1) {
            autoManager.setSpeed(.65);
            autoManager.runPath(autoManager.intakeYellow1, true);
            autoManager.safeSleep(350);
//            autoManager.holdPoint(autoManager.intakeYellow1.build().getPath(0).getLastControlPoint(), Math.toRadians(0), .5);
        } else if (sampleNum == 2) {
            autoManager.runPath(autoManager.intakeYellow2, true);
            autoManager.safeSleep(350);
//            autoManager.holdPoint(autoManager.intakeYellow2.build().getPath(0).getLastControlPoint(), Math.toRadians(0), .5);
        } else if (sampleNum == 3) {
            autoManager.setSpeed(.7);
            arm.setAutoLastSample(true);
            intake.setGrabAngle(GrabAngle.CUSTOM);
            intake.setCustomGrabAngle(135);
            autoManager.runPath(autoManager.intakeYellow3, true);
            autoManager.safeSleep(750);
//            autoManager.safeSleep(50000);
//            autoManager.holdPoint(autoManager.intakeYellow3.build().getPath(0).getLastControlPoint(), .875, .5);
        }
        //        autoManager.safeSleep(350);

        if(sampleNum == 3) {
            arm.intakeDownMode();
            autoManager.safeSleep(350);
        } else {
            autoManager.waitForSlides();
            autoManager.safeSleep(250);
        }

        intake.intake();
        autoManager.safeSleep(250);

        if (sampleNum == 1) {
            s1Xerror = follower.getPose().getX();
            s1Yerror = follower.getPose().getY();
        } else if (sampleNum == 2) {
            s2Xerror = follower.getPose().getX();
            s2Yerror = follower.getPose().getY();
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
        autoManager.safeSleep(850);
    }

    public void bucketScore(int sampleNum) {
        if (arm.getTeleopMode() != TeleopMode.BUCKET_SCORE) {
            currentMode = TeleopMode.BUCKET_SCORE;
            arm.setTeleopMode(currentMode);
            arm.setBucket(2);
            arm.update();
            if (sampleNum == 1) {
                autoManager.safeSleep(750);
            } else {
                autoManager.safeSleep(100);
            }
        }

        autoManager.setSpeed(.8);
        if (sampleNum == 1) {
            autoManager.runPath(autoManager.bucketScorePathS1, false);
        } else if (sampleNum == 2) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake1, true);
        } else if (sampleNum == 3) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake2, true);
        } else if (sampleNum == 4) {
            autoManager.runPath(autoManager.bucketScorePathFromIntake3, true);
        }

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        autoManager.safeSleep(200);
        arm.setArmTipBucketScore(true);
        autoManager.safeSleep(200);
        intake.outtake();
        autoManager.safeSleep(200);
        arm.setArmTipBucketScore(false);
        autoManager.safeSleep(450);

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
