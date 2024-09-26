package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
public class IntakeSubsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;

    public IntakeSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

    }   // close RRMechOps constructor Method

    public void intake() {
        robot.intakeServo.setPosition(params.INTAKE_SPEED);
    }

    public void outtake() {
        robot.intakeServo.setPosition(params.OUTTAKE_SPEED);
    }

    public void hold() {
        robot.intakeServo.setPosition(params.INTAKE_HOLD_SPEED);
    }

    public void idle() {
        robot.intakeServo.setPosition(params.INTAKE_IDLE_SPEED);
    }
}