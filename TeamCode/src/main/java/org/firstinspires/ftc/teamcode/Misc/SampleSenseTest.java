package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoRoadrunner.AutoHomeAction;
import org.firstinspires.ftc.teamcode.AutoRoadrunner.AutoManager;
import org.firstinspires.ftc.teamcode.AutoRoadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.OpenCV.SampleDetectionPipelinePNP;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "2: Sample Sense Test", group = "1")
@Config
public class SampleSenseTest extends LinearOpMode {
    SampleDetectionPipelinePNP detectionPipe;
    OpenCvWebcam webcam;
    MecanumDrive drive;
    double startX = 0;
    public static double kP = .1;
    public static double kI = 0.5;
    private boolean homed = true;
    public static double kD = 0.01;
    private PIDController drivePid = new PIDController(kP, kI, kD);
    AutoManager autoManager;
    private IntakeSubsystem intake;
    private HWProfile robot;
    private double ang = 0;


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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        detectionPipe = new SampleDetectionPipelinePNP();

        webcam.setPipeline(detectionPipe);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)), false);

        autoManager = new AutoManager(this);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
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
        waitForStart();

        while (opModeIsActive())
        {
            drive.updatePoseEstimate();

            /*
             * Send some stats to the telemetry
             */
//            telemetry.addData("Start X: ", startX);
//            telemetry.addData("Robot X: ", drive.pose.position.x);
//            telemetry.addData("Robot Y: ", drive.pose.position.y);
//            telemetry.addData("Robot Heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.addData("Frame Count", webcam.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
//            telemetry.addData("Samples: ", detectionPipe.getDetectedStones().size());
            if(detectionPipe.getDetectedStones().size() >= 1) {
                double x = detectionPipe.getDetectedStones().get(0).tvec.get(0,0)[0];
                double y = detectionPipe.getDetectedStones().get(0).tvec.get(1,0)[0];
                if(startX == 0) startX = x;

//                telemetry.addData("Sample 1 X: ", x);
//                telemetry.addData("Go to Sample X: ", drive.pose.position.x - (3 - x) * 0.254065041);
//                telemetry.addData("Go to Sample Y: ", drive.pose.position.y + (-5 - y) * 0.254065041);
//                telemetry.addData("Sample 1 Y: ", y);

                drive.updatePoseEstimate();
                autoManager.updatePose(drive.pose);
                autoManager.setDrive(drive);

                if(!homed) {
//                    autoManager.homeToSample(detectionPipe);
//                    Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(new Vector2d(drive.pose.position.x - (3 - x) * 0.254065041,drive.pose.position.y - (-5 + y) * 0.254065041), Math.toRadians(90))
//                                .build());
                    AutoHomeAction autoHomeAction = new AutoHomeAction(drive, webcam, detectionPipe, this);
                    while (!homed && opModeIsActive()) {
                        homed = !autoHomeAction.run(new TelemetryPacket());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(new Vector2d(5,5), Math.toRadians(90))
//                                .build());
                    }
                }

//                telemetry.addData("x error: ", drive.xError);
//                telemetry.addData("y error: ", drive.yError);

                ang = 0;
                ang = 90 - detectionPipe.getDetectedStones().get(0).angle;
//                ang = 90 - intake.getCustomGrabAngle() + detectionPipe.getDetectedStones().get(0).angle;

                intake.setGrabAngle(GrabAngle.CUSTOM);
                intake.setCustomGrabAngle(ang);
                intake.update();


                telemetry.addData("Sample 1 Angle: ", detectionPipe.getDetectedStones().get(0).angle);
                telemetry.addData("Sample 1 Target Ang: ", ang);
                telemetry.addData("Current Claw Angle: ", detectionPipe.getDetectedStones().get(0).angle);
            }
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */

            if(gamepad1.a)
            {
                homed = false;
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }
}