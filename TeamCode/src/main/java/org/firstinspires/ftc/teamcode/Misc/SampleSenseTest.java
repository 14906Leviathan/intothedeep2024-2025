package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoPedro.AutoManagerPedro;
import org.firstinspires.ftc.teamcode.AutoRoadrunner.AutoManager;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipelinePNP;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "2: Sample Sense Test", group = "1")
@Config
public class SampleSenseTest extends LinearOpMode {
    SampleDetectionPipelinePNP detectionPipe;
    OpenCvWebcam webcam;
    private Follower follower;
    double startX = 0;
    public static double kP = .001;
    public static double kI = 0;
    private boolean homed = false;
    private boolean runHome = false;
    public static double kD = 0;
    private PIDController drivePid = new PIDController(kP, kI, kD);
    AutoManagerPedro autoManager;
    private IntakeSubsystem intake;
    private HWProfile robot;
    private double currentTargetX = 0;
    private double ang = 0;
    private ArmSubsystem arm;
    private ElapsedTime timer;


    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        follower = new Follower(hardwareMap);

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update();
            intake.update();
        }, arm, intake, robot);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipe = new SampleDetectionPipelinePNP();

        webcam.setPipeline(detectionPipe);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        intake = new IntakeSubsystem(robot, this, new Params());

        /*
         * Wait for the user to press start on the Driver Station
         */

        arm = new ArmSubsystem(robot, this, new Params());
        arm.setAutoMode(true);
        arm.setPedroAuto(true);
        arm.setArmPower(1);
        arm.setSlidesPower(1);

        arm.setTeleopMode(TeleopMode.INTAKE);
        arm.setIntakePosition(new Params().PEDRO_AUTO_INTAKE_Y1_POS);
        arm.intakeUpMode();
        arm.update();

        while (opModeInInit()) {
            arm.update();
        }

        waitForStart();

        timer = new ElapsedTime();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            arm.update();
            intake.outtake();
            follower.update();

            /*
             * Send some stats to the telemetry
             */
//            telemetry.addData("Start X: ", startX);
//            telemetry.addData("Robot X: ", drive.pose.position.x);
//            telemetry.addData("Robot Y: ", drive.pose.position.y);
//            telemetry.addData("Robot Heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.addData("Frame Count", webcam.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));

            drivePid.setPID(kP, kI, kD);

            telemetry.addData("Samples: ", detectionPipe.getDetectedStones().size());
            if (detectionPipe.getDetectedStones().size() >= 1) {
                double x = detectionPipe.getDetectedStones().get(0).tvec.get(1, 0)[0]; // * -0.886367665;
                double y = detectionPipe.getDetectedStones().get(0).tvec.get(0, 0)[0]; //* 0.362568435;

                telemetry.addData("x: ", x);
                telemetry.addData("y: ", y);

                if (runHome && !follower.isBusy()) {
//                    double targetX = follower.getPose().getX() + (-5 - x);
//
////                    follower.followPath(new Path(
////                            new BezierLine(
////                                    new Point(follower.getPose()),
////                                    new Point(targetX, 0, Point.CARTESIAN)
////                            )
////                    )
////                            .setConstantHeadingInterpolation(0)
////                            .setPathEndTimeoutConstraint(.9), false);
//                    follower.holdPoint(new Pose(
//                            targetX,
//                            0,
//                            0
//                    ));
//
//                    follower.setMaxPower(.5);
//
//                    telemetry.addData("targetX: ", targetX);
//                    autoManager.homeToSample(detectionPipe, 0, 1000);
                }
            } else {
                follower.setTeleOpMovementVectors(0, 0, 0, true);
            }

            if (gamepad1.left_bumper) {
                arm.intakeDownMode();
            } else {
                arm.intakeUpMode();
            }

            telemetry.addData("robot x: ", follower.getPose().getX());
            telemetry.addData("robot y: ", follower.getPose().getY());

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */

            if (gamepad1.a) {
                runHome = true;
                homed = false;

                timer.reset();
            }

            if (timer.time(TimeUnit.MILLISECONDS) > 500) {
                homed = true;
                runHome = false;
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            telemetry.update();

            safeWait(100);
        }
    }

    public void safeWait(int time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.time(TimeUnit.MILLISECONDS) <= time) {
            follower.update();
            arm.update();
            intake.update();

            if (isStopRequested()) break;
            if (!opModeIsActive()) break;
        }
    }
}