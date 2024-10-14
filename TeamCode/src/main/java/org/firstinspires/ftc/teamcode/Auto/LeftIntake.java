package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Abstracts.AutoProgram;
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

public class LeftIntake extends AutoProgram {
    private LinearOpMode opMode;
    private Follower follower;
    private ElapsedTime autoTime = new ElapsedTime();
    private int autoState = 1;
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private long armWaitSleep = 2500;
    private long outtakeSleep = 1000;
    private long waitForHalt = 500;
    private long intakeSleep = 1000;
    private long waitIntakeDown = 1000;
    private long waitToGrab = 500;
    private boolean autoStart = true;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private AutoManager autoManager;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Thread armHandlerThread = new Thread(() -> {
        while (opMode.opModeIsActive()) {
            arm.update();
            intake.update();
            autoManager.update();

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
    private AutoLocation autoLocation = AutoLocation.LEFT_SCORE_GOLD;
    private int nextAutoState = 0;
    private final String autoName = "Left Intake Autonomous";

    public String getAutoName() {
        return autoName;
    }

    public LeftIntake() {
    }

    public void init(LinearOpMode _opMode) {
        opMode = _opMode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        follower = new Follower(hardwareMap);
        follower.resetIMU();
        follower.setPose(new Pose(9, 83.5, Math.toRadians(0)));
        telemetry.update();

        robot = new HWProfile();
        robot.init(hardwareMap, false);
        params = new Params();
        arm = new ArmSubsystem(robot, opMode, params);
        intake = new IntakeSubsystem(robot, opMode, params);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);

        intake.setGrabAngle(grabAngle);
        intake.setGrabStyle(grabStyle);
        intake.intake();

        intake.update();
        arm.update();

        autoManager = new AutoManager(follower);
        autoManager.buildPaths(autoLocation);

        while (!opMode.opModeIsActive()) arm.update();

        startAuto();
    }

    public void startAuto() {
        autoManager.setSpeed(params.AUTO_MAX_SPEED);

        while (opMode.opModeIsActive()) {
            if(!armHandlerThread.isAlive()) armHandlerThread.start();

            arm.setSlidesPower(params.SLIDE_MOTOR_POWER);

            if(debug) {
                autoManager.disablePathing(true);
                continue;
            }

            switch(autoState) {
                case 1:
                    if(!autoManager.isBusy()) {
                        autoManager.runPath(autoManager.toBucketPath);

                        autoState = 2;
                    }
                    break;
                case 2:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.BUCKET_SCORE;
                        arm.setTeleopMode(currentMode);
                        opMode.sleep(armWaitSleep);
                        autoManager.runPath(autoManager.bucketScorePath);
                        autoState = 3;
                    }
                    break;
                case 3:
                    if(!autoManager.isBusy()) {
//                        autoManager.holdCurrentPoint();
                        opMode.sleep(waitForHalt);
                        intake.outtake();
                        opMode.sleep(outtakeSleep);

                        autoManager.runPath(autoManager.backupPath);
                        autoManager.setSpeed(params.AUTO_MAX_SPEED);
                        autoState = 4;
                    } else {
                        autoManager.setSpeed(params.AUTO_OUTTAKE_SPEED);
                    }
                    break;
                case 4:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);
                        autoManager.setSpeed(params.AUTO_INTAKE_SPEED);
                        autoManager.runPath(autoManager.intakeYellow1);
                        autoState = 5;
                    }
                    break;
                case 5:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.setIntakePosition(params.AUTO_INTAKE_Y1_POS);

                        while (!arm.slidesAtPosition()) continue;
                        opMode.sleep(waitToGrab);

                        arm.intakeGrab();
                        opMode.sleep(waitIntakeDown);
                        intake.intake();

                        opMode.sleep(750);

                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);

                        autoManager.runPath(autoManager.toBucketPath);
                        autoState = 6;
                    }
                case 6:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.BUCKET_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.setBucket(2);
                        autoManager.runPath(autoManager.bucketScorePath);
                        autoManager.setSpeed(params.AUTO_OUTTAKE_SPEED);
                        autoState = 7;
                    }
                    break;
                case 7:
                    if(!autoManager.isBusy()) {
                        autoManager.setSpeed(params.AUTO_MAX_SPEED);
                        opMode.sleep(waitForHalt);
                        intake.outtake();
                        opMode.sleep(outtakeSleep);
                        autoManager.runPath(autoManager.backupPath);

                        autoState = 8;
                    }
                case 8:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);
                        autoManager.setSpeed(params.AUTO_INTAKE_SPEED);
//                        autoManager.runPath(autoManager.intakeYellow1);
                        autoState = 9;
                    }

                case 20:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        opMode.sleep(armWaitSleep);
                        autoManager.runPath(autoManager.park);
                        autoState = 21;
                    }
                    break;
                case 21:
                    if(!autoManager.isBusy()) {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        autoState = -1;
                    } else {
                        if(follower.getPose().getX() > 45) {
                            autoManager.setSpeed(params.AUTO_PARK_SPEED);
                        }
                    }
                    break;
                /*
                case 2:
                    if(!follower.isBusy()) {
                        currentMode = TeleopMode.BUCKET_SCORE;
                        arm.setBucket(2);
                        arm.setTeleopMode(currentMode);
                        sleep(armWaitSleep);
                        follower.followPath(bucketScorePath.getPath().build());

                        autoState = 3;
                    }
                break;
                case 3:
                    if(!follower.isBusy()) {
                        sleep(waitForHalt);
                        intake.outtake();
                        sleep(outtakeSleep);
                        follower.followPath(backupPath.getPath().build());
                    }
                break;
                */
            }

            arm.update();
        }

    }
}
