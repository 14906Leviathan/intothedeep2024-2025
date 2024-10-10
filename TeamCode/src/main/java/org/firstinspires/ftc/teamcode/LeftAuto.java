package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Auto;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
@Autonomous (name = "Left Auto", group = "Autonomous", preselectTeleOp = "Main TeleOp")
public class LeftAuto extends LinearOpMode {
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
    private boolean autoStart = true;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private Auto auto;
    private Thread armHandlerThread = new Thread(() -> {
       while (opModeIsActive()) {
           arm.update();
           intake.update();
           auto.update();

           telemetry.addData("X:", follower.getPose().getX());
           telemetry.addData("Y:", follower.getPose().getY());
           telemetry.addData("heading:", Math.toDegrees(follower.getTotalHeading()));
           telemetry.addData("auto state:", autoState);
           telemetry.update();
       }
    });
    private final boolean debug = false;
    private boolean pathStarted = true;
    private PathChain currentPath;
    private int lastAutoState = 0;
    private AutoLocation autoLocation = AutoLocation.LEFT_SIMPLE;
    private int nextAutoState = 0;

    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        follower.resetIMU();
        follower.setPose(new Pose(9, 83.5, Math.toRadians(0)));
        telemetry.update();

        robot = new HWProfile();
        robot.init(hardwareMap, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        currentMode = TeleopMode.IDLE;
        arm.setTeleopMode(currentMode);

        intake.setGrabAngle(grabAngle);
        intake.setGrabStyle(grabStyle);
        intake.intake();

        intake.update();
        arm.update();

        auto = new Auto(follower);
        auto.buildPaths(autoLocation);

        while (!opModeIsActive()) {
            arm.update();
        }

        while (opModeIsActive()) {
            if(!armHandlerThread.isAlive()) armHandlerThread.start();

            arm.setSlidesPower(params.SLIDE_MOTOR_POWER);

            /*
            if(debug) {
                auto.update();

                if(autoStart) {
                    autoStart = false;

                    auto.runPath(auto.toBucketPath);
                }

                continue;
            }

             */

            auto.disablePathing(debug);


//            if(auto.isBusy()) {
//                auto.update();
//                continue;
//            }

            switch(autoState) {
                case 1:
                    if(!auto.isBusy()) {
                        auto.runPath(auto.toBucketPath);

                        autoState = 2;
                    }
                break;
                case 2:
                    if(!auto.isBusy()) {
                        currentMode = TeleopMode.BUCKET_SCORE;
                        arm.setTeleopMode(currentMode);
                        sleep(armWaitSleep);
                        auto.runPath(auto.bucketScorePath);
                        autoState = 3;
                    }
                break;
                case 3:
                    if(!auto.isBusy()) {
//                        auto.holdCurrentPoint();
                        sleep(waitForHalt);
                        intake.outtake();
                        sleep(outtakeSleep);

                        auto.runPath(auto.backupPath);
                        auto.setSpeed(.75);
                        autoState = 4;
                    } else {
                        auto.setSpeed(.35);
                    }
                break;
                case 4:
                    if(!auto.isBusy()) {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        sleep(armWaitSleep);
                        auto.runPath(auto.park);
                        autoState = 5;
                    }
                break;
                case 5:
                    if(!auto.isBusy()) {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        autoState = -1;
                    } else {
                        if(follower.getPose().getX() > 45) {
                            auto.setSpeed(.35);
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
