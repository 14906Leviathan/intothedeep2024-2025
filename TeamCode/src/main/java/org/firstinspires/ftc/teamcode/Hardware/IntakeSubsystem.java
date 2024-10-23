package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeMode;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

@Config
public class IntakeSubsystem extends Subsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    public IntakeMode currentIntakeMode = IntakeMode.HOLD;
    public GrabStyle grabStyle;
    public GrabAngle grabAngle;
    public boolean closed = false;

    public IntakeSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

        grabStyle = GrabStyle.OUTSIDE_GRAB;
        grabAngle = GrabAngle.VERTICAL_GRAB;
        intake();
        closed = true;
    }

    public void setGrabStyle(GrabStyle _grabStyle) {
        grabStyle = _grabStyle;
    }

    public void setGrabAngle(GrabAngle _grabAngle) {
        grabAngle = _grabAngle;
    }

    public GrabStyle getGrabStyle() {
        return grabStyle;
    }

    public GrabAngle getGrabAngle() {
        return grabAngle;
    }

    public void toggle() {
        if(closed) {
            outtake();
            closed = false;
        } else {
            intake();
            closed = true;
        }
    }

    public void intake() {
        currentIntakeMode = IntakeMode.INTAKE;
        closed = true;
    }

    public void outtake() {
        currentIntakeMode = IntakeMode.OUTTAKE;
        closed = false;
    }

    public void hold() {
        currentIntakeMode = IntakeMode.HOLD;
    }

    public void idle() {
        currentIntakeMode = IntakeMode.IDLE;
    }

    public void update() {
        if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
            if (currentIntakeMode == IntakeMode.INTAKE) {
                robot.intakeServo1.setPower(params.INTAKE_SPEED);
                robot.intakeServo2.setPower(params.INTAKE_SPEED);

                robot.intakeServo1.setDirection(params.INTAKE1_DIRECTION);
                robot.intakeServo2.setDirection(params.INTAKE2_DIRECTION);
            } else if (currentIntakeMode == IntakeMode.IDLE) {
                robot.intakeServo1.setPower(params.INTAKE_IDLE_SPEED);
                robot.intakeServo2.setPower(params.INTAKE_IDLE_SPEED);
            } else if (currentIntakeMode == IntakeMode.OUTTAKE) {
                robot.intakeServo1.setPower(params.OUTTAKE_SPEED);
                robot.intakeServo2.setPower(params.OUTTAKE_SPEED);

                robot.intakeServo1.setDirection(params.OUTAKE1_DIRECTION);
                robot.intakeServo2.setDirection(params.OUTAKE2_DIRECTION);
            } else if (currentIntakeMode == IntakeMode.HOLD) {
                robot.intakeServo1.setPower(params.INTAKE_HOLD_SPEED);
                robot.intakeServo2.setPower(params.INTAKE_HOLD_SPEED);

                robot.intakeServo1.setDirection(params.INTAKE1_DIRECTION);
                robot.intakeServo2.setDirection(params.INTAKE2_DIRECTION);
            }
        } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
            if(currentIntakeMode == IntakeMode.INTAKE) {
                if(grabStyle == GrabStyle.OUTSIDE_GRAB) {
                    robot.clawServo.turnToAngle(params.CLAW_OUTSIDE_GRAB_ANGLE);
                } else if(grabStyle == GrabStyle.INSIDE_GRAB) {
                    robot.clawServo.turnToAngle(params.CLAW_INSIDE_GRAB_ANGLE);
                }
            } else if(currentIntakeMode == IntakeMode.OUTTAKE) {
                if(grabStyle == GrabStyle.OUTSIDE_GRAB) {
                    robot.clawServo.turnToAngle(params.CLAW_OUTSIDE_DROP_ANGLE);
                } else if(grabStyle == GrabStyle.INSIDE_GRAB) {
                    robot.clawServo.turnToAngle(params.CLAW_INSIDE_DROP_ANGLE);
                }
            }

            if(grabAngle == GrabAngle.VERTICAL_GRAB) {
                robot.clawPivotServo.turnToAngle(params.PIVOT_VERTICAL_ANG);
            } else if(grabAngle == GrabAngle.HORIZONTAL_GRAB) {
                robot.clawPivotServo.turnToAngle(params.PIVOT_HORIZONTAL_ANG);
            }
        }
    }

    public void setTeleopMode(TeleopMode mode) {
        return;
    }
}