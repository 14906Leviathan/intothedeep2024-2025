package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

@Config
public class IntakeSubsystem extends Subsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    public IntakeMode currentIntakeMode = IntakeMode.HOLD;

    public IntakeSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;
    }

    public void intake() {
        currentIntakeMode = IntakeMode.INTAKE;
    }

    public void outtake() {
        currentIntakeMode = IntakeMode.OUTTAKE;
    }

    public void hold() {
        currentIntakeMode = IntakeMode.HOLD;
    }

    public void idle() {
        currentIntakeMode = IntakeMode.IDLE;
    }

    public void update() {
        if(currentIntakeMode == IntakeMode.INTAKE) {
            robot.intakeServo1.setPower(params.INTAKE_SPEED);
            robot.intakeServo2.setPower(params.INTAKE_SPEED);

            robot.intakeServo1.setDirection(params.INTAKE1_DIRECTION);
            robot.intakeServo2.setDirection(params.INTAKE2_DIRECTION);
        } else if(currentIntakeMode == IntakeMode.IDLE) {
            robot.intakeServo1.setPower(params.INTAKE_IDLE_SPEED);
            robot.intakeServo2.setPower(params.INTAKE_IDLE_SPEED);
        } else if(currentIntakeMode == IntakeMode.OUTTAKE) {
            robot.intakeServo1.setPower(params.OUTTAKE_SPEED);
            robot.intakeServo2.setPower(params.OUTTAKE_SPEED);

            robot.intakeServo1.setDirection(params.OUTAKE1_DIRECTION);
            robot.intakeServo2.setDirection(params.OUTAKE2_DIRECTION);
        } else if(currentIntakeMode == IntakeMode.HOLD) {
            robot.intakeServo1.setPower(params.INTAKE_HOLD_SPEED);
            robot.intakeServo2.setPower(params.INTAKE_HOLD_SPEED);

            robot.intakeServo1.setDirection(params.INTAKE1_DIRECTION);
            robot.intakeServo2.setDirection(params.INTAKE2_DIRECTION);
        }
    }

    public void setTeleopMode(TeleopMode mode) {
        return;
    }
}