package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoPedro.AutoManagerPedro;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
@TeleOp()
public class LimelightTest extends LinearOpMode {
    private HWProfile robot = new HWProfile();
//    private MecanumDrive drive;
    public static double kP = 0.08;
    public static double kI = 0;
    public static double kD = 0;
    private PIDController xController = new PIDController(kP, kI, kD);
    private PIDController yController = new PIDController(kP, kI, kD);
    private Follower follower;
    private ArmSubsystem arm;
    public static double clampLimit = .1;
    private boolean home = false;
    private double holdX = 0;
    private AutoManagerPedro autoManager;
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false, false);

//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)), true);

        arm = new ArmSubsystem(robot, this, new Params());
        intake = new IntakeSubsystem(robot, this, new Params());

        follower = new Follower(hardwareMap);

        autoManager = new AutoManagerPedro(this, follower, () -> {
            arm.update();
            intake.update();
        }, arm, intake, robot);

        arm.setTeleopMode(TeleopMode.INTAKE);
        arm.setAutoMode(true);
        arm.setPedroAuto(true);
        arm.intakeUpMode();
        arm.setArmPower(1);
        arm.setSlidesPower(1);
        arm.setIntakePosition(new Params().PEDRO_AUTO_INTAKE_Y1_POS);
        arm.update();

        robot.limelight.start();

        while (opModeInInit()) {
            arm.update();
        }

        waitForStart();

        follower.startTeleopDrive();


        while (opModeIsActive()) {
            LLResult result = robot.limelight.getLatestResult();

            arm.setIntakePosition(new Params().PEDRO_AUTO_INTAKE_Y1_POS);
            arm.update();

//            if(result != null) {
//                if(result.isValid()) {
//                    Pose3D botpose = result.getBotpose_MT2();
//
//                    telemetry.addData("pose x: ", (botpose.getPosition().x * -27.2479564) - 33.5369);
//                    telemetry.addData("pose y: ", (botpose.getPosition().y * 21.32196) + 50.9578);
//                }
//            }

            xController.setSetPoint(2);
            yController.setSetPoint(14);
            xController.setPID(kP, kI, kD);
            yController.setPID(kP, kI, kD);

            if(result != null) {
                if (result.getTx() != 0 && result.getTy() != 0) {
//                    LLResultTypes.DetectorResult sample = result.getDetectorResults().get(0);

                    double x = (result.getPythonOutput()[0] - 320 / 2);
                    double y = (result.getPythonOutput()[1] - 320 / 2);

                    double xOut = xController.calculate(x);

                    telemetry.addData("xOut: ", xOut);

                    xOut = MathFunctions.clamp(xOut, -clampLimit, clampLimit);
                    double targetX = follower.getPose().getX() + (5.7 - x);

                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);

                    if(gamepad1.a) {
//                        home = true;
//
//                        follower.holdPoint(new Pose(
//                                targetX,
//                                follower.getPose().getY(),
//                                0
//                        ));
//
//                        holdX = targetX;
                        autoManager.homeToSample(0);
                    }

                    if(home) {
                        if((Math.abs(holdX - follower.getPose().getX()) < .25) && (Math.abs(5.7 - x) > .25)) {
                            follower.holdPoint(new Pose(
                                    targetX,
                                    follower.getPose().getY(),
                                    0
                            ));

                            holdX = targetX;
                        }
                    }

                    if(gamepad1.left_bumper) {
                        arm.intakeDownMode();
                    } else {
                        arm.intakeUpMode();
                    }

//                    follower.setTeleOpMovementVectors(xOut, 0, 0);
                    follower.setMaxPower(.35);
//                    follower.holdPoint(new Pose(x - 51, y - 14, 0));
//                    sleep(100);
                } else {
                    follower.setTeleOpMovementVectors(0, 0, 0);
                }

                telemetry.addData("staleness: ", result.getStaleness());
                telemetry.addData("result: ", result.getDetectorResults().size());
                telemetry.addData("barcode: ", result.getBarcodeResults().size());
                telemetry.addData("x: ", result.getPythonOutput()[0]);
                telemetry.addData("y: ", result.getPythonOutput()[1]);
                telemetry.addData("tx: ", result.getTx());
                telemetry.addData("ty: ", result.getTy());
                telemetry.addData("ta: ", result.getTa());
                telemetry.addLine();
                telemetry.addData("pipeline: ", result.getPipelineIndex());
            } else {
                follower.setTeleOpMovementVectors(0, 0, 0);
            }

            if(gamepad1.b) {
                follower.setPose(new Pose(0, 0, follower.getTotalHeading()));
            }

            follower.update();

            telemetry.addData("robot x: ", follower.getPose().getX());
            telemetry.addData("robot y: ", follower.getPose().getY());
            telemetry.addData("fps: ", robot.limelight.getStatus().getFps());
            telemetry.update();
        }
    }

}
