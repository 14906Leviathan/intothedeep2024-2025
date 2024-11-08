package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name = "1: Reset Slide Encoder", group = "1")
public class ResetSlideEncoder extends LinearOpMode {
    private HWProfile robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HWProfile();
        robot.init(hardwareMap, false, false);

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("P: ", robot.slidesMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
            telemetry.addData("I: ", robot.slidesMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
            telemetry.addData("D: ", robot.slidesMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
            telemetry.update();

            robot.slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
