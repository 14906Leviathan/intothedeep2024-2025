//package org.firstinspires.ftc.teamcode.AutoRoadrunner;
//
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Abstracts.AutoProgram;
//import org.firstinspires.ftc.teamcode.Misc.AprilTagDrive;
//import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
//import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
//import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
//import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
//import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
//import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Hardware.Params;
//import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;
//
//public class LeftIntakeThreeRR_ATAG extends AutoProgram {
//    private LinearOpMode opMode;
//    private MecanumDrive drive;
//    private ElapsedTime autoTime = new ElapsedTime();
//    private int autoState = 1;
//    private HWProfile robot;
//    private Params params;
//    private ArmSubsystem arm;
//    private IntakeSubsystem intake;
//    private TeleopMode currentMode = TeleopMode.IDLE;
//    private long armWaitSleep = 750;
//    private long outtakeSleep = 600;
//    private long waitForArmIntakeDown = 500;
//    private long waitToIntake = 1250;
//    private long waitForGrab = 150;
//    private long waitForHalt = 500;
//    private long intakeSleep = 1000;
//    private boolean autoStart = true;
//    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
//    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
//    private AutoManager autoManager;
//    private HardwareMap hardwareMap;
//    private Telemetry telemetry;
//    private int threadCycle = 0;
//    private boolean ltopmodewasstart = false;
//    private Thread armHandlerThread = new Thread(() -> {
//        while (opMode.opModeIsActive()) {
//            arm.update();
//            intake.update();
//
//            if(arm.teleopModeStart) ltopmodewasstart = true;
//
//            telemetry.addData("teleopModeStartWorked:", ltopmodewasstart);
//            telemetry.addData("slides transistion:", arm.armTransistionStage);
//            telemetry.addData("X:", drive.pose.position.x);
//            telemetry.addData("Arm pos:", arm.getArmPosition());
//            telemetry.addData("Slides pos:", arm.getSlidesPosition());
//            telemetry.addData("slides at pos:", arm.slidesAtPosition());
//            telemetry.addData("thread cycle:", threadCycle);
//            telemetry.addData("Y:", drive.pose.position.y);
//            telemetry.addData("heading:", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.update();
//        }
//    });
//    private final boolean debug = false;
//    private boolean pathStarted = true;
//    private int lastAutoState = 0;
//    private AutoLocation autoLocation = AutoLocation.LEFT_SCORE_THREE_GOLD_ATAG;
//    private int nextAutoState = 0;
//    private final String autoName = "Left Intake Autonomous New ATAG (TEMPORARY)";
//    private TrajectoryActionBuilder currentPath;
//    private InstantAction updateAction;
//
//    public String getAutoName() {
//        return autoName;
//    }
//
//    public LeftIntakeThreeRR_ATAG() {
//
//    }
//
//    public void init(LinearOpMode _opMode) {
//        opMode = _opMode;
//        telemetry = opMode.telemetry;
//        hardwareMap = opMode.hardwareMap;
//
//        telemetry.update();
//
//        robot = new HWProfile();
//        robot.init(hardwareMap, false);
//        params = new Params();
//        arm = new ArmSubsystem(robot, opMode, params);
//        intake = new IntakeSubsystem(robot, opMode, params);
//
//        currentMode = TeleopMode.IDLE;
//        arm.setTeleopMode(currentMode);
//        arm.setAutoMode(true);
//        arm.poleToucherIn();
//
//        intake.setGrabAngle(grabAngle);
//        intake.setGrabStyle(grabStyle);
//        intake.intake();
//
//        intake.update();
//        arm.update();
//
//        arm.resetSlidesPosition();
//
//        autoManager = new AutoManager(opMode);
//        drive = new AprilTagDrive(hardwareMap, new Pose2d(autoManager.XOffset,autoManager.YOffset,Math.toRadians(90)), autoManager.getATAGProcessor());
//
//        drive.updatePoseEstimate();
//
//        autoManager.setDrive(drive);
//        robot.slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (!opMode.opModeIsActive()) {
//            arm.update();
//            telemetry.addData("X:", drive.pose.position.x);
//            telemetry.addData("slides raw enc:", robot.slidesMotor.getCurrentPosition());
//            telemetry.addData("Y:", drive.pose.position.y);
//            telemetry.addData("heading:", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.update();
//        }
//
//
//        startAuto();
//    }
//
//    public void intakePath(double intakePosition, boolean quickArm) {
//        arm.setArmPower(params.ARM_POWER_DEFAULT);
//        arm.setSlidesPower(1);
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.backupPath = autoManager.backupPath
////                .afterTime(.35, new InstantAction(() -> {
////                    if(quickArm) {
////                        grabAngle = GrabAngle.HORIZONTAL_GRAB;
////                        intake.setGrabAngle(grabAngle);
////
////                        currentMode = TeleopMode.DOWN;
////                        arm.setTeleopMode(currentMode);
////                        arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
////                        arm.intakeUpMode();
////                    }
////                }))
////                .strafeToLinearHeading(new Vector2d(20, 0), Math.toRadians(45));
////        autoManager.runPath(autoManager.backupPath);
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        drive.extraCorrection = true;
//        if(intakePosition == 1) {
//            drive.extraCorrection = false;
//            intake.setShortRange(false);
//
//            drive.updatePoseEstimate();
//            currentPath = drive.actionBuilder(drive.pose)
//                .afterTime(.75, new InstantAction(() -> {
//                    if(quickArm) {
//                        grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                        intake.setGrabAngle(grabAngle);
//
//                        currentMode = TeleopMode.DOWN;
//                        arm.setTeleopMode(currentMode);
//                        arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//                        arm.intakeUpMode();
//                    }
//                }))
//                .strafeToLinearHeading(autoManager.intakeYellow1StrafeTo, autoManager.intakeYellow1StrafeToHeading)
//                .afterTime(0, new InstantAction(() -> {
//                    drive.extraCorrection = true;
//                }))
//                .strafeToLinearHeading(autoManager.intakeYellow1Path, autoManager.intakeYellow1PathHeading);
//            autoManager.runPath(currentPath);
//        } else if(intakePosition == 2) {
//            intake.setShortRange(false);
//            drive.updatePoseEstimate();
//
//            currentPath = drive.actionBuilder(drive.pose)
//                    .afterTime(.75, new InstantAction(() -> {
//                        if(quickArm) {
//                            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                            intake.setGrabAngle(grabAngle);
//
//                            currentMode = TeleopMode.DOWN;
//                            arm.setTeleopMode(currentMode);
//                            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//                            arm.intakeUpMode();
//                        }
//                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow2Path, autoManager.intakeYellow2PathHeading);
//            autoManager.runPath(currentPath);
//        } else if(intakePosition == 3) {
//            intake.setShortRange(true);
//            intake.outtake();
//            intake.update();
//
//            drive.extraCorrection = false;
//
//            drive.updatePoseEstimate();
//            currentPath = drive.actionBuilder(drive.pose)
//                    .afterTime(.75, new InstantAction(() -> {
//                        if(quickArm) {
//                            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//                            intake.setGrabAngle(grabAngle);
//
//                            currentMode = TeleopMode.DOWN;
//                            arm.setTeleopMode(currentMode);
//                            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//                            arm.intakeUpMode();
//                        }
//                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow3StrafeTo, autoManager.intakeYellow3StrafeToHeading)
////                    .afterTime(0, new InstantAction(() -> {
////                        drive.extraCorrection = true;
////                    }))
//                    .strafeToLinearHeading(autoManager.intakeYellow3Path, autoManager.intakeYellow3PathHeading);
//            autoManager.runPath(currentPath);
//        }
//
//        if(!quickArm) {
//            grabAngle = GrabAngle.HORIZONTAL_GRAB;
//            intake.setGrabAngle(grabAngle);
//
//            currentMode = TeleopMode.DOWN;
//            arm.setTeleopMode(currentMode);
//            arm.setIntakePosition(params.THREE_AUTO_INTAKE_Y1_POS);
//            arm.intakeUpMode();
////        arm.setArmPower(params.ARM_POWER_SLOW_AUTO);
//            arm.update();
//            waitForArm();
//        }
//        drive.extraCorrection = false;
//
//        opMode.sleep(100);
//        arm.intakeDownMode();
//        opMode.sleep(waitForArmIntakeDown);
////        waitForArm();
//        intake.intake();
//        opMode.sleep(waitForGrab);
//
//        if(quickArm) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update();
//            opMode.sleep(100);
//        } else {
//            currentMode = TeleopMode.IDLE;
//            arm.setTeleopMode(currentMode);
//            opMode.sleep(armWaitSleep);
//        }
//    }
//
//    public void bucketScore(boolean goToStandby, boolean quickArm) {
//        grabAngle = GrabAngle.VERTICAL_GRAB;
//        intake.setGrabAngle(grabAngle);
//
//        if(quickArm) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update();
//            opMode.sleep(100);
//        }
//
//        if(goToStandby) {
//            autoManager.updatePose(drive.pose);
//            autoManager.buildPaths(autoLocation);
//            autoManager.runPath(autoManager.toBucketPath);
//        }
//
//        if(!quickArm) {
//            currentMode = TeleopMode.BUCKET_SCORE;
//            arm.setTeleopMode(currentMode);
//            arm.setBucket(2);
//            arm.update();
//            opMode.sleep(armWaitSleep);
//        }
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//        autoManager.runPath(autoManager.bucketScorePathFromIntake1);
//
//        arm.setArmTipBucketScore(true);
//        opMode.sleep(outtakeSleep);
//        intake.outtake();
//        arm.setArmTipBucketScore(false);
//        opMode.sleep(500);
//    }
//
//    public void startAuto() {
//        if(!armHandlerThread.isAlive()) armHandlerThread.start();
//
//        updateAction = new InstantAction(() -> {
////            arm.update();
////            intake.update();
//        });
//
//        drive.setUpdateAction(updateAction);
//
//        arm.poleToucherOut();
//        arm.setArmPower(params.ARM_POWER_DEFAULT);
//
//        autoManager.updatePose(drive.pose);
//        autoManager.buildPaths(autoLocation);
//
//        arm.setSlidesPower(params.SLIDE_MOTOR_POWER);
//        autoManager.disablePathing(debug);
//        arm.update();
//
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.runPath(autoManager.toBucketPath);
//
//        currentMode = TeleopMode.BUCKET_SCORE;
//        arm.setTeleopMode(currentMode);
//        arm.setBucket(2);
//        arm.update();
//        opMode.sleep(100);
//
//        drive.updatePoseEstimate();
//        currentPath = drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(autoManager.strafeToBucket, autoManager.strafeToBucketHeading);
//        autoManager.runPath(currentPath);
//
//        bucketScore(false, false);
//        intakePath(1, true);
//        bucketScore(true, true);
//        intakePath(2, true);
//        bucketScore(true, true);
//        intakePath(3, true);
//        bucketScore(true, true);
//
////        autoManager.updatePose(drive.pose);
////        autoManager.buildPaths(autoLocation);
////        autoManager.runPath(autoManager.toBucketPath);
////
////        currentMode = TeleopMode.AUTO_SLIDES_IN;
////        arm.setTeleopMode(currentMode);
//
//        opMode.sleep(10000);
//
//    }
//
//    public void waitForArm() {
//        while (!arm.armAtPosition() && !arm.slidesAtPosition()) drive.updatePoseEstimate(); arm.update();
//    }
//}