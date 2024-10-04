package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Paths.BlueLeft.*;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
@Autonomous (name = "Park Auto", group = "Autonomous", preselectTeleOp = "Main TeleOp")
public class BlueLeftAuto extends LinearOpMode {
    private boolean forward = true;
    private Follower follower;
    private ElapsedTime autoTime = new ElapsedTime();
    private ToBucketPath toBucketPath;
    private BucketScorePath bucketScorePath;
    private ToFirstYellowPath toFirstYellowPath;
    private int autoState = 1;
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private long armWaitSleep = 2500;
    private long outtakeSleep = 2500;
    private long intakeSleep = 3500;
    private Thread armHandlerThread = new Thread(() -> {
       while (opModeIsActive()) {
           arm.update();
           intake.update();

           telemetry.addData("X:", follower.getPose().getX());
           telemetry.addData("Y:", follower.getPose().getY());
           telemetry.addData("heading:", Math.toDegrees(follower.getTotalHeading()));
           telemetry.update();
       }
    });

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

        waitForStart();

        while (opModeIsActive()) {
            if(!armHandlerThread.isAlive()) armHandlerThread.start();

            switch(autoState) {
                case 1:
                    follower.followPath(toBucketPath.getPath().build());

                    autoState = 2;
                break;
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
                        currentMode = TeleopMode.INTAKE;
                        intake.outtake();
                        sleep(outtakeSleep);
                        arm.setTeleopMode(currentMode);
                        arm.setIntakePosition(params.INTAKE_MIN_POS);
                        sleep(armWaitSleep);
                        autoState = 4;
                        follower.followPath(toFirstYellowPath.getPath().build());
                    }
                break;
                case 4:
                    if(!follower.isBusy()) {
                        arm.setIntakePosition(params.INTAKE_MAX_POS);
                        intake.intake();
                        sleep(intakeSleep);
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);

                        follower.followPath(toBucketPath.getPath().build());
                        autoState = 5;
                    }
                break;
                case 5:
                    if(!follower.isBusy()){
                        currentMode = TeleopMode.BUCKET_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.setBucket(2);
                        sleep(armWaitSleep);
                        follower.followPath(bucketScorePath.getPath().build());

                        autoState = 6;
                    }
                break;
                case 6:
                    if(!follower.isBusy()) {
                        intake.outtake();
                        sleep(outtakeSleep);
                    }
            }

            follower.update();
            arm.update();
        }
    }
}
