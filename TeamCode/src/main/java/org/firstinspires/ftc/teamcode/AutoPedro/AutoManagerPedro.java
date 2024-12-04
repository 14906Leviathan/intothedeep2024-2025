package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.AutoLocation;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;

@Config
public class AutoManagerPedro {
    public PathBuilder toBucketPath;
    //    public PathBuilder backupPath;
    public PathBuilder turnPath;
    public PathBuilder bucketScorePathFromIntake1;
    public PathBuilder bucketScorePathFromIntake2;
    public PathBuilder bucketScorePathFromIntake3;
    public PathBuilder bucketScorePathS1;
    public PathBuilder park;
    public PathBuilder intakeYellow1;
    public PathBuilder intakeYellow2;
    public PathBuilder intakeYellow3;
    public PathBuilder specScore1;
    public PathBuilder specScore2;
    public PathBuilder specScore3;
    public PathBuilder intakeSpec1;
    public PathBuilder pushSample1;
    public PathBuilder intakeSpec2;
    public PathBuilder specBackup;
    private Follower follower;
    private HWProfile robot;
    private ArmSubsystem arm;
    private TeleopMode currentMode = TeleopMode.IDLE;
    private IntakeSubsystem intake;
    private Point startPose = new Point(0, 0, Point.CARTESIAN);
    private LinearOpMode opMode;
    private boolean pathingDisabled = false;
    // zero is two tiles from bucket to the leftmost of the tile
    public Pose start_4_0_V1 = new Pose(.399, -.48, Math.toRadians(90));
    public Pose start_3_1_V1 = new Pose(0, -23.6, Math.toRadians(0));
    public Pose start_0_3_V1 = new Pose(0, -49.9, Math.toRadians(0));
    private Point leftScoreLocation = new Point(19, 109, Point.CARTESIAN);
    private Runnable updateAction;
    private MultipleTelemetry telemetry;
    private Params params;
    public static double xP = .05;
    public static double xI = 0;
    public static double xD = 0;
    private PIDController xController = new PIDController(xP, xI, xD);

    public AutoManagerPedro(LinearOpMode opMode, Follower _follower, Runnable updateAction, ArmSubsystem arm, IntakeSubsystem intake, HWProfile robot) {
        follower = _follower;
        this.opMode = opMode;
        this.updateAction = updateAction;
        this.arm = arm;
        this.intake = intake;
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.robot = robot;
        this.params = new Params();
    }

    private void run(PathChain path, boolean correct) {
        if (!pathingDisabled) follower.followPath(path, correct);

        while (follower.isBusy()) {
            update();

            if (opMode.isStopRequested()) {
                follower.breakFollowing();
                break;
            }
        }
    }

    public Pose homeToSample(double heading, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double startY = follower.getPose().getY();
        boolean start = true;
        double holdX = 0;
        double targetX = 0;
        double targetY = 0;
        boolean usingPID = false;
        boolean usingPIDStart = true;
        boolean runFollow = true;
        boolean firstRun = true;
        boolean finalUpdate = false;
        double error = .5;
        double setPower = .35;
        int cycle = 0;
        ElapsedTime timer2 = new ElapsedTime();
        int runs = 1;
        double x = 0;
        double y = 0;
        double cps = 0;

        xController.setSetPoint(targetX);
        xController.setTolerance(1);

        timer2.reset();

        while (timer.time(TimeUnit.MILLISECONDS) < timeout) {
            xController.setPID(xP, xI, xD);

//            if (detectionPipe.getDetectedStones().size() == 1) {
//                double x = detectionPipe.getDetectedStones().get(0).tvec.get(1, 0)[0] * -0.886367665;
//                double y = detectionPipe.getDetectedStones().get(0).tvec.get(0, 0)[0] * 0.362568435;
//
//                telemetry.addData("x: ", x);
//
//                double targetX = follower.getPose().getX() + (-6 - x);
//
////                    follower.followPath(new Path(
////                            new BezierLine(
////                                    new Point(follower.getPose()),
////                                    new Point(targetX, 0, Point.CARTESIAN)
////                            )
////                    )
////                            .setConstantHeadingInterpolation(0)
////                            .setPathEndTimeoutConstraint(.9), false);
//                follower.holdPoint(new Pose(
//                        targetX,
//                        startY,
//                        heading
//                ));
//
//                follower.setMaxPower(1);
//                telemetry.addData("targetX: ", targetX);
//
//                update();
//
//                if (!opMode.opModeIsActive()) break;
//                if (opMode.isStopRequested()) break;
//
//                safeSleep(100);
//            } else {
//                follower.holdPoint(new Pose(
//                        follower.getPose().getX(),
//                        follower.getPose().getY(),
//                        heading
//                ));
//            }
            LLResult result = robot.limelight.getLatestResult();

            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;

            if(timer2.time(TimeUnit.SECONDS) >= 1) {
                cps = cycle;

                timer2.reset();
                cycle = 0;
            }

            if (result != null) {
                if (result.isValid()) {
                    cycle++;

                    robot.limelight.reloadPipeline();

                    follower.setMaxPower(1);

                    LLResultTypes.DetectorResult sample = result.getDetectorResults().get(0);

                    x = sample.getTargetYDegrees() * 0.15060241;
                    y = sample.getTargetXDegrees() * -0.182815356;
                    double xToPos = (follower.getPose().getX() + (targetX - x));
                    double yToPos = (follower.getPose().getY() + (targetY - y));

//                    if(x < 0) xToPos /= 2;

                    try {
                        telemetry.addData("update hold: ", Math.abs(follower.getPose().getX() - follower.getCurrentPath().getLastControlPoint().getX()) < .35 && Math.abs(targetX - x) > error);
                    } catch (Exception e) {
                        //boo hoo an error threw
                    }
                    telemetry.addData("runFollow: ", runFollow);
                    telemetry.addData("x: ", x);
                    telemetry.addData("x velo: ", follower.getVelocity().getXComponent());
                    telemetry.addData("xToPos: ", xToPos);
                    telemetry.addData("fps: ", robot.limelight.getStatus().getFps());
                    telemetry.addData("y: ", y);

                    telemetry.addData("cycles per seconds: ", cps);

//                    usingPID = Math.abs(x) >= 3;
//                    usingPID = false;


                    if (runFollow) {

                        if(cps >= 10) {
//                            safeSleep(250);
//                            continue;
                        }

                        follower.holdPoint(new Pose(
                                xToPos,
                                yToPos,
                                heading
                        ));

                        runFollow = false;
                    }

                    if ((/*Math.abs(follower.getVelocity().getXComponent()) < .5 && */Math.abs(targetX - x) > 1) || Math.abs(targetY - y) > 1) {
                        follower.updateHoldPoint(new Pose(
                                xToPos,
                                yToPos,
                                heading
                        ));

                        safeSleep(350);
                    }
                } else {
                    follower.setMaxPower(.1);
                }
            } else {
                follower.setMaxPower(.1);

                telemetry.addLine("low power");
            }

            telemetry.update();
        }

        follower.startTeleopDrive();
        follower.setTeleOpMovementVectors(0, 0, 0);

        return new Pose(x, y);
    }

    public void runPath(PathChain path) {
        run(path, true);
    }

    public void runPath(PathBuilder builder) {
        run(builder.build(), true);
    }

    public void runPath(PathChain path, boolean correct) {
        run(path, correct);
    }

    public void runPath(PathBuilder builder, boolean correct) {
        run(builder.build(), correct);
    }

    public void update() {
        robot.limelight.getLatestResult();

        follower.update();
        updateAction.run();

//        telemetry.addData("get closest T: ", follower.getCurrentTValue());
//        telemetry.addData("get path type: ", follower.getCurrentPath().pathType());
////            telemetry.update();
        try {
            follower.telemetryDebug(telemetry);
        } catch (Exception e) {

        }
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void disablePathing(boolean disabled) {
        pathingDisabled = disabled;
    }

    public void holdCurrentPoint() {
        BezierPoint currentPoint = new BezierPoint(follower.getCurrentPath().getLastControlPoint());
        follower.holdPoint(currentPoint, follower.getCurrentPath().getPathEndHeadingConstraint());
    }

    public void setSpeed(double speed) {
        follower.setMaxPower(speed);
    }

    public void holdPoint(Pose pose) {
        follower.holdPoint(pose);
    }

    public void buildPaths(AutoLocation autoLocation) {
        if (autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1 || autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
            toBucketPath = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(0, 0, Point.CARTESIAN),
                                    new Point(15, 9, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140));

//            toBucketPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140));
            double startHeading = 0;

            if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
                startPose = new Point(start_3_1_V1.getX(), start_3_1_V1.getY(), Point.CARTESIAN);
                startHeading = start_3_1_V1.getHeading();

                specScore1 = new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        startPose,
                                        new Point(33 /*2.75*/, -45/*19.5*/, Point.CARTESIAN),
                                        new Point(33 /*2.75*/, -40/*19.5*/, Point.CARTESIAN)
                                )
                        ))
                        .addTemporalCallback(0, () -> {
                            currentMode = TeleopMode.SPECIMEN_SCORE;
                            arm.setTeleopMode(currentMode);
                            arm.update();

                            arm.setArmPositionSpecimen(100);
                            arm.setSlidesPositionSpecimen(8);
                        })
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setPathEndTimeoutConstraint(250)
                        .setPathEndTValueConstraint(.9);

                specBackup = new PathBuilder()
                        .addPath(new Path(
                                new BezierCurve(
                                        specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                        new Point(0, 3, Point.CARTESIAN),
                                        new Point(16, 6, Point.CARTESIAN)
                                )
                        ))
                        .addParametricCallback(.35, () -> {
                            currentMode = TeleopMode.INTAKE;
                            arm.setTeleopMode(currentMode);
                            arm.update();
                        })
                        .addParametricCallback(.99, () -> {
//                            relocalizeX();
                        })
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .setPathEndTimeoutConstraint(100)
                        .setPathEndTValueConstraint(.85);
            } else if (autoLocation == AutoLocation.PEDRO_LEFT_4_0_V1) {
                startPose = new Point(start_4_0_V1.getX(), start_4_0_V1.getY(), Point.CARTESIAN);
                startHeading = start_4_0_V1.getHeading();
            }

            Point bucketFinalPoint = new Point(2.75 /*2.75*/, 20/*19.5*/, Point.CARTESIAN);

            bucketScorePathS1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    startPose,
                                    new Point(3 /*2.75*/, 20/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(startHeading, Math.toRadians(130))
                    .setPathEndTValueConstraint(.9);

//            bucketScorePathFromIntake1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            bucketScorePathFromIntake1.getPath(0).setPathEndTValueConstraint(.9);

//            backupPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierLine(
//                                    bucketScorePathFromIntake1.build().getPath(0).getLastControlPoint(),
//                                    new Point(9, 11, Point.CARTESIAN)
//                            )
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(140));

//            backupPath.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            backupPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));


//            turnPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierPoint(
//                                    backupPath.getPath(backupPath.size() - 1).getLastControlPoint()
//                            )
//                    )).build();
//
//            turnPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));

            Point intakeYellow1Start = bucketFinalPoint;
            double intakeYellow1StartHeading = 130;
            double xOffset = -1;

            if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V1) {
                intakeYellow1Start = specBackup.build().getPath(0).getLastControlPoint();
                intakeYellow1StartHeading = 0;
                xOffset = 0;
            }


            intakeYellow1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1Start,
                                    new Point(19 - xOffset, 10, Point.CARTESIAN) //24.25 10
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(intakeYellow1StartHeading), Math.toRadians(0))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                    })
                    .addParametricCallback(.9, () -> {
                        setSpeed(1);
                    })
                    .setPathEndTValueConstraint(.95)
                    .setPathEndTimeoutConstraint(100);

//            intakeYellow1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow1.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow1.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(17 - xOffset, 20, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))
//                    .addPath(new Path(
//                            new BezierLine(
//                                    new Point(17, 20, Point.CARTESIAN),
//                                    new Point(17.1, 20, Point.CARTESIAN)
//                            )
//                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTValueConstraint(.97)
                    .setPathEndTimeoutConstraint(100);

//            intakeYellow2.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow2.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow2.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(28.1 - xOffset, 12, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(130), .875, .25)
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                    })
                    .addParametricCallback(.9, () -> {
                        setSpeed(1);
                    })
                    .setPathEndTValueConstraint(.97)
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(26.5, 12, Point.CARTESIAN),
                                    new Point(26.5, 20, Point.CARTESIAN)
                            )
                    ))
                    .setConstantHeadingInterpolation(.875)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), .875)
                    .setPathEndTValueConstraint(.93)
                    .setPathEndTimeoutConstraint(100);

            bucketScorePathFromIntake1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1.build().getPath(intakeYellow1.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow2.build().getPath(intakeYellow2.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow3.build().getPath(intakeYellow3.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(.875, Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

//            intakeYellow3.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), .875);
//            intakeYellow3.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow3.getPath(0).setsetAniPathEndTimeoutConstraint(500);

            park = new PathBuilder()
                    .addPath(new Path(
                                    new BezierLine(
                                            bucketFinalPoint,
                                            new Point(54, 1, Point.CARTESIAN)
                                    )
                            )
                    )
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                        arm.setParkArmUp(true);
                        safeSleep(1500);
                        arm.setParkArmUp(false);
                        safeSleep(10000);
                    })

                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(-90))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(54, 1, Point.CARTESIAN),
                                    new Point(54, -12.5, Point.CARTESIAN)
                            )
                    )).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(100);

//            park.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90));
//            park.getPath(1).setConstantHeadingInterpolation(Math.toRadians(-90));

//            park.getPath(0).setPathEndTValueConstraint(.9);
//            park.getPath(1).setPathEndTValueConstraint(.9);
//            park.getPath(0).setPathEndTimeoutConstraint(10);
//            park.getPath(1).setPathEndTimeoutConstraint(10);
        } else if (autoLocation == AutoLocation.PEDRO_LEFT_0_3_V1) {
            startPose = new Point(start_0_3_V1.getX(), start_0_3_V1.getY(), Point.CARTESIAN);

            specScore1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    startPose,
                                    new Point(25 /*2.75*/, -46/*19.5*/, Point.CARTESIAN),
                                    new Point(34.5 /*2.75*/, -45/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(0, () -> {
                        currentMode = TeleopMode.SPECIMEN_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.update();

                        arm.setArmPositionSpecimen(100);
                        arm.setSlidesPositionSpecimen(8);
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(250)
                    .setPathEndTValueConstraint(.9);

            pushSample1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                    new Point(0, -30, Point.CARTESIAN),
                                    new Point(10, -70, Point.CARTESIAN)
                            )
                    ))
                    .setConstantHeadingInterpolation(0)
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(10, -70, Point.CARTESIAN),
                                    new Point(50, -70, Point.CARTESIAN),
                                    new Point(50, -80, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(0, () -> {
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                    })
                    .setConstantHeadingInterpolation(0)
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(50, -80, Point.CARTESIAN),
                                    new Point(10, -80, Point.CARTESIAN)
                            )
                    ))
                    .setConstantHeadingInterpolation(0)
                    .setPathEndTValueConstraint(.9);

            intakeSpec1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    pushSample1.build().getPath(pushSample1.build().size() - 1).getLastControlPoint(),
                                    new Point(25 /*2.75*/, -80/*19.5*/, Point.CARTESIAN),
                                    new Point(5.5 /*2.75*/, -92/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.35, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.intakeSpecimen = true;
                        arm.intakeUpMode();
                        arm.update();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135), .5)
                    .setPathEndTimeoutConstraint(250)
                    .setPathEndTValueConstraint(.95);

            specScore2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec1.build().getPath(intakeSpec1.build().size() - 1).getLastControlPoint(),
                                    new Point(10 /*2.75*/, -80/*19.5*/, Point.CARTESIAN),
                                    new Point(34.5 /*2.75*/, -35/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.35, () -> {
                        currentMode = TeleopMode.SPECIMEN_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.update();

                        arm.setArmPositionSpecimen(100);
                        arm.setSlidesPositionSpecimen(8);
                    })
                    .setLinearHeadingInterpolation(intakeSpec1.build().getPath(intakeSpec1.build().size() - 1).getHeadingGoal(1), Math.toRadians(0))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.9);

            intakeSpec2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore2.build().getPath(specScore2.build().size() - 1).getLastControlPoint(),
                                    new Point(10 /*2.75*/, -60/*19.5*/, Point.CARTESIAN),
                                    new Point(6 /*2.75*/, -91/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.35, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.intakeSpecimen = true;
                        arm.intakeUpMode();
                        arm.update();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135), .5)
                    .setPathEndTimeoutConstraint(250)
                    .setPathEndTValueConstraint(.95);

            specScore3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    intakeSpec2.build().getPath(intakeSpec2.build().size() - 1).getLastControlPoint(),
                                    new Point(0 /*2.75*/, -80/*19.5*/, Point.CARTESIAN),
                                    new Point(38 /*2.75*/, -25/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.35, () -> {
                        currentMode = TeleopMode.SPECIMEN_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.update();

                        arm.setArmPositionSpecimen(100);
                        arm.setSlidesPositionSpecimen(8);
                    })
                    .setLinearHeadingInterpolation(intakeSpec1.build().getPath(intakeSpec1.build().size() - 1).getHeadingGoal(1), Math.toRadians(0))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.9);

            park = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore3.build().getPath(specScore3.build().size() - 1).getLastControlPoint(),
                                    new Point(5, -40, Point.CARTESIAN),
                                    new Point(0, -90, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.IDLE;
                        arm.setTeleopMode(currentMode);
                        arm.setAnimationType(AnimationType.NONE);
                        arm.update();
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(0)
                    .setPathEndTValueConstraint(.9);
        } else if (autoLocation == AutoLocation.PEDRO_LEFT_3_1_V2) {
            toBucketPath = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(0, 0, Point.CARTESIAN),
                                    new Point(15, 9, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140));

//            toBucketPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140));
            double startHeading = start_3_1_V1.getHeading();

            startPose = new Point(start_3_1_V1.getX(), start_3_1_V1.getY(), Point.CARTESIAN);

            specScore1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    startPose,
                                    new Point(33 /*2.75*/, -45/*19.5*/, Point.CARTESIAN),
                                    new Point(33 /*2.75*/, -40/*19.5*/, Point.CARTESIAN)
                            )
                    ))
                    .addTemporalCallback(0, () -> {
                        currentMode = TeleopMode.SPECIMEN_SCORE;
                        arm.setTeleopMode(currentMode);
                        arm.update();

                        arm.setArmPositionSpecimen(100);
                        arm.setSlidesPositionSpecimen(8);
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(250)
                    .setPathEndTValueConstraint(.9);

            specBackup = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    specScore1.build().getPath(specScore1.build().size() - 1).getLastControlPoint(),
                                    new Point(0, 3, Point.CARTESIAN),
                                    new Point(15, 10, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.35, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.setIntakePush(true);
                        arm.update();

                        intake.intake();
                        intake.update();
                    })
                    .addParametricCallback(.99, () -> {
//                            relocalizeX();
                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTimeoutConstraint(100)
                    .setPathEndTValueConstraint(.95);

            Point bucketFinalPoint = new Point(2.75 /*2.75*/, 20/*19.5*/, Point.CARTESIAN);
//            bucketScorePathFromIntake1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            bucketScorePathFromIntake1.getPath(0).setPathEndTValueConstraint(.9);

//            backupPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierLine(
//                                    bucketScorePathFromIntake1.build().getPath(0).getLastControlPoint(),
//                                    new Point(9, 11, Point.CARTESIAN)
//                            )
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(140));

//            backupPath.getPath(0).setConstantHeadingInterpolation(Math.toRadians(140));
//            backupPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));


//            turnPath = new PathBuilder()
//                    .addPath(new Path(
//                            new BezierPoint(
//                                    backupPath.getPath(backupPath.size() - 1).getLastControlPoint()
//                            )
//                    )).build();
//
//            turnPath.getPath(0).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0));


            Point intakeYellow1Start = specBackup.build().getPath(0).getLastControlPoint();
            double intakeYellow1StartHeading = 0;

            pushSample1 = new PathBuilder()
                    .addPath(new BezierLine(
                            specBackup.build().getPath(0).getLastControlPoint(),
                            new Point(26, 10, Point.CARTESIAN) //24.25 10
                    ))
                    .setConstantHeadingInterpolation(0)
                    .setPathEndTValueConstraint(.99)
                    .setPathEndTimeoutConstraint(0);


            intakeYellow1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierCurve(
                                    new Point(15, 10, Point.CARTESIAN), //24.25 10
                                    new Point(5, 10, Point.CARTESIAN), //24.25 10
                                    new Point(29, 10, Point.CARTESIAN) //24.25 10
                            )
                    ))
                    .setPathEndTValueConstraint(.99)
                    .setPathEndTimeoutConstraint(100)
                    .setConstantHeadingInterpolation(0);

//            intakeYellow1.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow1.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow1.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(17, 20, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.6, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(17, 20, Point.CARTESIAN),
                                    new Point(23.3, 20, Point.CARTESIAN)
                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setPathEndTValueConstraint(.97)
                    .setPathEndTimeoutConstraint(100);

//            intakeYellow2.getPath(0).setConstantHeadingInterpolation(Math.toRadians(0));
//            intakeYellow2.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow2.getPath(0).setPathEndTimeoutConstraint(500);

            intakeYellow3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    bucketFinalPoint,
                                    new Point(28.1, 12, Point.CARTESIAN)
                            )
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(130), .875, .25)
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.INTAKE;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                    })
                    .addParametricCallback(.9, () -> {
                        setSpeed(1);
                    })
                    .setPathEndTValueConstraint(.97)
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(26.5, 12, Point.CARTESIAN),
                                    new Point(26.5, 20, Point.CARTESIAN)
                            )
                    ))
                    .setConstantHeadingInterpolation(.875)
//                    .setLinearHeadingInterpolation(Math.toRadians(0), .875)
                    .setPathEndTValueConstraint(.93)
                    .setPathEndTimeoutConstraint(100);

            bucketScorePathFromIntake1 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow1.build().getPath(intakeYellow1.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake2 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow2.build().getPath(intakeYellow2.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

            bucketScorePathFromIntake3 = new PathBuilder()
                    .addPath(new Path(
                            new BezierLine(
                                    intakeYellow3.build().getPath(intakeYellow3.build().size() - 1).getLastControlPoint(),
                                    bucketFinalPoint

                            )
                    ))
                    .addParametricCallback(.95, () -> {
                        setSpeed(1);
                    })
                    .setLinearHeadingInterpolation(.875, Math.toRadians(130), .5)
//                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .setPathEndTValueConstraint(.87);

//            intakeYellow3.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), .875);
//            intakeYellow3.getPath(0).setPathEndTValueConstraint(.99);
//            intakeYellow3.getPath(0).setsetAniPathEndTimeoutConstraint(500);

            park = new PathBuilder()
                    .addPath(new Path(
                                    new BezierLine(
                                            bucketFinalPoint,
                                            new Point(54, 1, Point.CARTESIAN)
                                    )
                            )
                    )
                    .addParametricCallback(.4, () -> {
                        currentMode = TeleopMode.TOUCH_POLE_AUTO;
                        arm.setTeleopMode(currentMode);
                        arm.update();
                        arm.setParkArmUp(true);
                        safeSleep(1500);
                        arm.setParkArmUp(false);
                        safeSleep(10000);
                    })

                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(-90))
                    .addPath(new Path(
                            new BezierLine(
                                    new Point(54, 1, Point.CARTESIAN),
                                    new Point(54, -12.5, Point.CARTESIAN)
                            )
                    )).setConstantHeadingInterpolation(Math.toRadians(-90))
                    .setPathEndTValueConstraint(.9)
                    .setPathEndTimeoutConstraint(100);

//            park.getPath(0).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90));
//            park.getPath(1).setConstantHeadingInterpolation(Math.toRadians(-90));

//            park.getPath(0).setPathEndTValueConstraint(.9);
//            park.getPath(1).setPathEndTValueConstraint(.9);
//            park.getPath(0).setPathEndTimeoutConstraint(10);
//            park.getPath(1).setPathEndTimeoutConstraint(10);
        }


    }

    public void safeSleep(int sleep) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (sleep >= time.time(TimeUnit.MILLISECONDS)) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    public void waitForSlides() {
        double position = arm.getSlidesTargetPos();

        while (!(arm.getSlidesPosition() + this.params.SLIDES_ERROR_TOLERANCE_AUTO > position && arm.getSlidesPosition() - this.params.SLIDES_ERROR_TOLERANCE_AUTO < position)) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    public void waitForArm() {
        double targetPosition = arm.getArmTargetPosition();
        double position = arm.getArmPosition();

        while (!(position + this.params.ARM_ERROR_TOLERANCE_AUTO > targetPosition && position - this.params.ARM_ERROR_TOLERANCE_AUTO < targetPosition)) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    public void waitForArmAndSlides(int timeout) {
        double position = arm.getSlidesTargetPos();
        ElapsedTime timer = new ElapsedTime();
        double armPos = arm.getArmPosition();
        double armTargetPos = arm.getArmTargetPosition();
        double slidesPos = arm.getSlidesPosition();
        double slidesTargetPos = arm.getSlidesTargetPos();
        timer.reset();

        while (!((armPos + this.params.ARM_ERROR_TOLERANCE_AUTO > armTargetPos && position - this.params.ARM_ERROR_TOLERANCE_AUTO < armTargetPos) && (armPos + this.params.SLIDES_ERROR_TOLERANCE_AUTO > armTargetPos && armPos - this.params.SLIDES_ERROR_TOLERANCE_AUTO < armTargetPos)) || timer.time(TimeUnit.MILLISECONDS) > timeout) {
            update();

            if (opMode.isStopRequested()) break;
            if (!opMode.opModeIsActive()) break;
        }
    }

    private void relocalizeX() {
        follower.setPose(new Pose(robot.distanceOne.getDistance(DistanceUnit.INCH) - .25, follower.getPose().getY(), follower.getTotalHeading()));
    }
}