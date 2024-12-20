package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoRoadrunner.AutoManager;
import org.firstinspires.ftc.teamcode.AutoRoadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.Params;

@TeleOp(name = "SpecHomeTest")
public class SpecHomeTest extends LinearOpMode {
    private HWProfile robot = new HWProfile();
    private Params params = new Params();
    private AutoManager autoManager;
    private MecanumDrive drive;
    private ArmSubsystem arm;

    public void runOpMode() {
        robot.init(hardwareMap, false, false);
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)), false);
        autoManager = new AutoManager(this);
        arm = new ArmSubsystem(robot, this, params);

        autoManager.setDrive(drive);

        arm.setTeleopMode(TeleopMode.INTAKE);
        arm.intakeSpecimen = true;
        arm.intakeUpSpecimen = 1;
        arm.setSlidesPower(1);
        arm.resetSlidesPosition();
//        arm.update();

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
//                telemetry.addData("distance: ", robot.distanceOne.getDistance(DistanceUnit.INCH));
//                telemetry.update();
//                arm.update();
            }
        }).start();

        while (opModeIsActive()) {
            autoManager.setDrive(drive);

            autoManager.homeToSpecimen(robot);
//            requestOpModeStop();
        }
    }
}
