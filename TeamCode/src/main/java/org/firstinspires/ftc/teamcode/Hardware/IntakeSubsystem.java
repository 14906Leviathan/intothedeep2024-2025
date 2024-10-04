package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        robot.intakeServo1.setPower(params.INTAKE_SPEED);
        robot.intakeServo2.setPower(params.INTAKE_SPEED);

        robot.intakeServo1.setDirection(params.INTAKE1_DIRECTION);
        robot.intakeServo2.setDirection(params.INTAKE2_DIRECTION);
    }

    public void outtake() {
        robot.intakeServo1.setPower(params.OUTTAKE_SPEED);
        robot.intakeServo2.setPower(params.OUTTAKE_SPEED);

        robot.intakeServo1.setDirection(params.OUTAKE1_DIRECTION);
        robot.intakeServo2.setDirection(params.OUTAKE2_DIRECTION);
    }

    public void hold() {
        robot.intakeServo1.setPower(params.INTAKE_HOLD_SPEED);
        robot.intakeServo2.setPower(params.INTAKE_HOLD_SPEED);

        robot.intakeServo1.setDirection(params.INTAKE1_DIRECTION);
        robot.intakeServo2.setDirection(params.INTAKE2_DIRECTION);
    }

    public void idle() {
        robot.intakeServo1.setPower(params.INTAKE_IDLE_SPEED);
        robot.intakeServo2.setPower(params.INTAKE_IDLE_SPEED);
    }
}