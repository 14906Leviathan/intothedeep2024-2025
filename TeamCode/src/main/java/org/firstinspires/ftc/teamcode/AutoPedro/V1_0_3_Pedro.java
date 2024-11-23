package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "0+3 V1 Pedro", group = "0", preselectTeleOp = "0: Main TeleOp")
public class V1_0_3_Pedro extends LinearOpMode {
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
    public static boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.PEDRO_LEFT_0_3_V1;
    private int nextAutoState = 0;
    private Telemetry telemetryA;
    private Pose startPose;

    public V1_0_3_Pedro() {
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
            telemetryA.addData("distance: ", robot.distanceOne.getDistance(DistanceUnit.INCH));
            telemetryA.addData("x: ", follower.getPose().getX());
            telemetryA.addData("y: ", follower.getPose().getY());
            telemetryA.update();
        }, arm, intake, robot);
        autoManager.buildPaths(autoLocation);

        startPose = autoManager.start_0_3_V1;

        follower.setPose(startPose);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);
        intake.intake();

        autoManager.safeSleep(350);

        arm.setSlidesPower(1);

        while (!opModeIsActive()) {
            arm.update();
            intake.update();
            telemetry.addLine("The robot must be started on the fourth tile");
            telemetry.addLine("to the right from the buckets, the robot must be");
            telemetry.addLine("started on the left line of the tile, and the robot");
            telemetry.addLine("needs to be facing towards the submersible");
            telemetry.addLine();
            telemetry.addData("slides raw enc:", robot.slidesMotor.getCurrentPosition());
            follower.setStartingPose(startPose);
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.update();
        }

        waitForStart();
        follower.updatePose();
        follower.update();
        autoManager.safeSleep(100);
        follower.setPose(startPose);
        autoManager.safeSleep(100);

        params.AUTO_SCORE = 37;
        params.AUTO_END_HEADING = 0;

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

        if (debug) {
            autoManager.safeSleep(1000000000);
        } else {

//        bucketScore(1);
            specimenScore(1);
            push(1);
            intakeSpec(1);
            specimenScore(2);
            intakeSpec(2);
            specimenScore(3);
            park();

            params.AUTO_END_HEADING = Math.toDegrees(follower.getTotalHeading());

            telemetryA.addData("time: ", time.time(TimeUnit.SECONDS));
            telemetryA.update();
            autoManager.safeSleep(10000);
        }
    }

    public void push(int sampleNum) {
        currentMode = TeleopMode.IDLE;

        if(sampleNum == 1) {
            autoManager.setSpeed(1);
            autoManager.runPath(autoManager.pushSample1, false);
            autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
        }
    }

    public void specimenScore(int specNum) {
        autoManager.setSpeed(.7);
        if (specNum == 1) {
            currentMode = TeleopMode.SPECIMEN_SCORE;
            autoManager.runPath(autoManager.specScore1);
        } else if (specNum == 2) {
            currentMode = TeleopMode.SPECIMEN_SCORE;
            autoManager.runPath(autoManager.specScore2);
        } else if (specNum == 3) {
            currentMode = TeleopMode.SPECIMEN_SCORE;
            autoManager.runPath(autoManager.specScore3);
        }
        autoManager.setSpeed(1);
        autoManager.safeSleep(400);
        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);

        arm.setArmPositionSpecimen(params.ARM_SCORE_SPECIMEN);
        arm.setSlidesPositionSpecimen(params.SLIDES_SCORE_SPECIMEN);
        autoManager.safeSleep(500);
//        arm.setSlidesPositionSpecimen(params.SLIDES_ENSURE_SCORE_SPECIMEN);
        autoManager.safeSleep(350);

        intake.outtake();
        intake.update();
        autoManager.safeSleep(650);
    }

    public void park() {
        autoManager.setSpeed(1);
        params.TELEOP_START_MODE = TeleopMode.IDLE;
        autoManager.runPath(autoManager.park, false);
//        arm.update();
//        autoManager.safeSleep(750);
//        arm.setParkArmUp(false);
//        autoManager.safeSleep(10000);
    }

    public void intakeSpec(int specNum) {
        intake.setShortRange(true);
        currentMode = TeleopMode.INTAKE;

        if (specNum == 1) {
            autoManager.setSpeed(.7);
            autoManager.runPath(autoManager.intakeSpec1, true);
            autoManager.safeSleep(350);
//            autoManager.holdPoint(autoManager.intakeYellow1.build().getPath(0).getLastControlPoint(), Math.toRadians(0), .5);
        } else if (specNum == 2) {
            autoManager.runPath(autoManager.intakeSpec2, true);
            autoManager.safeSleep(350);
//            autoManager.holdPoint(autoManager.intakeYellow2.build().getPath(0).getLastControlPoint(), Math.toRadians(0), .5);
        }
        //        autoManager.safeSleep(350);

        arm.intakeDownMode();
        autoManager.waitForArmAndSlides(1500);
        autoManager.safeSleep(750);

        intake.intake();
        autoManager.safeSleep(250);
        arm.intakeUpMode();
        autoManager.safeSleep(250);
//        autoManager.waitForArmAndSlides(750);

        if (specNum == 1) {
            s1Xerror = follower.getPose().getX();
            s1Yerror = follower.getPose().getY();
        } else if (specNum == 2) {
            s2Xerror = follower.getPose().getX();
            s2Yerror = follower.getPose().getY();
        } else if (specNum == 3) {
            s3Xerror = follower.getPose().getX();
            s3Yerror = follower.getPose().getY();
        }

        autoManager.setSpeed(params.AUTO_DEFAULT_SPEED);
    }
}
