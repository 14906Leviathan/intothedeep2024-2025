package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
public class ArmSubsystem extends Subsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    private TeleopMode currentMode;
    /*
    P: if you’re not where you want to be, get there.
    I: if you haven’t been where you want to be for a long time, get there faster
    D: if you’re getting close to where you want to be, slow down.
     */
    public static double Kp = 70;
    public static double Kp_Intake = 150;
    public static double Ki = .5;
    public static double Kd = .1;
    public static double Kf = 0.005;
    private double out = 0;
    private double oldTargetPos = 0;
    private double slidesPower = 0;
    private int bucketScore = 2;
    private PIDController armPID = new PIDController(Kp, Ki, Kd);
    public boolean intakeGrab = false;
    private boolean teleopModeStart = false;
    private double intakePos = 0;
    private double slidesTargetPos = 0;
    private boolean slidesRetract = false;
    private boolean waitSlidesTransition = false;
    private double armTargetPos = 0;
    private boolean armReachedPosition = false;
    private int armTransistionStage = 0;
    public boolean intakeSpecimen = false;
    public boolean intakeUpSpecimen = false;
    private double armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
    private double slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
    private TeleopMode lastTeleopMode = TeleopMode.IDLE;
    private boolean outtakeSlidesRetracted = false;
    private boolean armTipBucketScore = false;

    public void setTeleopMode(TeleopMode mode) {
        if(lastTeleopMode != mode) lastTeleopMode = currentMode;
        currentMode = mode;
        teleopModeStart = true;
        armReachedPosition = false;
    }

    public void setArmTipBucketScore(boolean set) {
        armTipBucketScore = set;
    }

    /*
     * Constructor method
     */
    public ArmSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;
    }   // close RRMechOps constructor Method

    public void poleToucherIn() {
        robot.poleToucher.turnToAngle(params.POLE_TOUCHER_IN);
    }

    public void poleToucherOut() {
        robot.poleToucher.turnToAngle(params.POLE_TOUCHER_OUT);
    }

    public double getArmPosition() {
        return (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_ABS_TICK_PER_DEG;
    }

    public void setArmTargetPosition(double deg) {
        if(waitSlidesTransition == false) armTargetPos = deg;
        opMode.telemetry.addData("(DEBUG: From ArmSubsystem) waitSlidesTransition:", waitSlidesTransition);
    }

    private void setArmPosition() {
        if(waitSlidesTransition == false) armPID.setSetPoint(armTargetPos);
    }

    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    public double getSlidesPosition() {
        return robot.slidesMotor.getCurrentPosition() / params.SLIDES_TICKS_PER_INCH;
    }

    public void setSlidesPower(double newPower) {
        slidesPower = newPower;
    }

    private void setTargetSlidesPosition(double len) {
        slidesTargetPos = len;

        setSlidesPosition(slidesTargetPos);
    }

    public void retractSlidesOuttake() {
        outtakeSlidesRetracted = true;
    }

    public void extendSlidesOuttake() {
        outtakeSlidesRetracted = false;
    }

    private void setSlidesPosition(double len) {
        len = MathFunctions.clamp(len, params.SLIDES_MIN_POS, params.SLIDES_MAX_POS);

        robot.slidesMotor.setPower(slidesPower);
        if(slidesRetract || waitSlidesTransition) {
            len = params.SLIDES_TRANSITION_LEN * params.SLIDES_TICKS_PER_INCH;
        } else {
            len = len * params.SLIDES_TICKS_PER_INCH;
        }

        robot.slidesMotor.setTargetPosition((int) len);
    }

    public void setIntakePosition(double len) {
        intakePos = len;
    }

    public void intakeGrab() {
        intakeGrab = true;
    }

    public double setArmPositionSpecimen(double pos) {
        if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
            pos = MathFunctions.clamp(pos, params.ARM_SPECIMEN_POLE_2_MIN_TWO_WHEEL, params.ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL);
        } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
            pos = MathFunctions.clamp(pos, params.ARM_SPECIMEN_POLE_2_MIN_CLAW, params.ARM_SPECIMEN_POLE_2_MAX_CLAW);
        }

        armPosSpecimen = pos;
        return armPosSpecimen;
    }

    public double setSlidesPositionSpecimen(double pos) {
        if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
            pos = MathFunctions.clamp(pos, params.SLIDES_SPECIMEN_POLE_2_MIN_TWO_WHEEL, params.SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL);
        } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
            pos = MathFunctions.clamp(pos, params.SLIDES_SPECIMEN_POLE_2_MIN_CLAW, params.SLIDES_SPECIMEN_POLE_2_MAX_CLAW);
        }

        slidesPosSpecimen = pos;

        return slidesPosSpecimen;
    }

    public void intakeUp() {
        intakeGrab = false;
    }

    public boolean armAtPosition() {
        double armPos = getArmPosition();

        return (armPos + params.ARM_AT_POS_ERROR_DEG > armTargetPos && armPos - params.ARM_AT_POS_ERROR_DEG < armTargetPos);
    }

    public boolean slidesAtPosition() {
        double slidesPos = getSlidesPosition();

        return (slidesPos + params.SLIDES_AT_POS_ERROR_INCH > slidesTargetPos && slidesPos - params.SLIDES_AT_POS_ERROR_INCH < slidesTargetPos);
    }

    public void setBucket(int bucket) {
        bucketScore = bucket;
    }

    public double getIntakePosition() {
        return Math.sqrt((getSlidesPosition()*getSlidesPosition()) - (params.SLIDE_GROUND_POS*params.SLIDE_GROUND_POS));
    }

    public double getArmTargetPosition() {
        return armPID.getSetPoint();
    }

    public void update() {
        if(currentMode == TeleopMode.INTAKE) {
            setSlidesPower(params.SLIDE_MOTOR_POWER);
            double slidesPos = MathFunctions.clamp(intakePos, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);

            slidesPos = Math.sqrt((params.SLIDE_GROUND_POS*params.SLIDE_GROUND_POS)+(slidesPos*slidesPos));

            if(!intakeSpecimen) {
                setTargetSlidesPosition(slidesPos);
                opMode.telemetry.addData("slidesPosArmClass: ", slidesPos);
                double armDeg = 0;
                if(params.INTAKE_TYPE == IntakeType.CLAW) armDeg = map(slidesPos, 3, 36, 17-4, 25-2);
                if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) armDeg = map(slidesPos, 3, 36, 17, 25);
                if (armDeg < 0) armDeg = 0;

                if (!intakeGrab && params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) armDeg += params.ARM_INTAKE_MODE_UP_DEG;
                if (!intakeGrab && params.INTAKE_TYPE == IntakeType.CLAW) armDeg += params.ARM_CLAW_MODE_UP_DEG;

                opMode.telemetry.addData("(DEBUG: From ArmSubsystem) armDeg:", armDeg);

                setArmTargetPosition(armDeg);
            } else {
                if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    setTargetSlidesPosition(params.SLIDES_SPECIMEN_INTAKE_TWO_WHEEL);
                    if (!intakeUpSpecimen) {
                        setArmTargetPosition(params.ARM_SPECIMEN_INTAKE_TWO_WHEEL);
                    } else {
                        setArmTargetPosition(params.ARM_SPECIMEN_INTAKE_TWO_WHEEL + 20);
                    }
                } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
                    setTargetSlidesPosition(params.SLIDES_SPECIMEN_INTAKE_CLAW);
                    setArmTargetPosition(params.ARM_SPECIMEN_INTAKE_CLAW);
                }
            }

            if(teleopModeStart) waitSlidesTransition = true;
            if(teleopModeStart) armReachedPosition = false;
            if(teleopModeStart) armTransistionStage = 1;

            if(armTransistionStage == 1) {
                slidesRetract = true;
                if(getSlidesPosition() >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    armTransistionStage = 2;
                }
            } else if(armTransistionStage == 2) {
                slidesRetract = true;
                waitSlidesTransition = false;
                if(armAtPosition()) {
                    armTransistionStage = 3;
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if (currentMode == TeleopMode.IDLE) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(params.ARM_IDLE_DEG);
            setSlidesPower(params.SLIDE_MOTOR_POWER);

            if(teleopModeStart) waitSlidesTransition = true;
            if(teleopModeStart) armReachedPosition = false;
            if(teleopModeStart) armTransistionStage = 1;

            if(lastTeleopMode != TeleopMode.INTAKE) {
                if (armTransistionStage == 1) {
                    slidesRetract = true;
                    if (getSlidesPosition() >= params.SLIDES_TRANSITION_LEN) {
                        waitSlidesTransition = true;
                    } else {
                        armTransistionStage = 2;
                    }
                } else if (armTransistionStage == 2) {
                    slidesRetract = true;
                    waitSlidesTransition = false;
                    if (armAtPosition()) {
                        armTransistionStage = 3;
                    }
                } else {
                    slidesRetract = false;
                    armReachedPosition = true;
                }
            } else {
                armReachedPosition = true;
                waitSlidesTransition = false;
                armTransistionStage = 3;
            }
        } else if (currentMode == TeleopMode.BUCKET_SCORE) {
            setSlidesPower(params.SLIDE_MOTOR_POWER_OUTTAKE);

            if(bucketScore == 2) {
                if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if(outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN);
                    }
                    setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
                } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
                    if(outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        if(armTipBucketScore) {
                            setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW - 5);
                        } else {
                            setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW);

                        }
                    }
                    if(armTipBucketScore) {
                        setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG - 5);
                    } else {
                        setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);

                    }
                }
            } else if(bucketScore == 1) {
                if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if(outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN);
                    }
                    setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
                } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
                    if(outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN_CLAW);
                    }
                    setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
                }
            }

            if(getArmPosition() < 90) {
                if (teleopModeStart) waitSlidesTransition = true;
                if (teleopModeStart) armReachedPosition = false;
                if (teleopModeStart) armTransistionStage = 1;


                if (armTransistionStage == 1) {
                    slidesRetract = true;
                    if (getSlidesPosition() >= params.SLIDES_TRANSITION_LEN) {
                        waitSlidesTransition = true;
                    } else {
                        armTransistionStage = 2;
                    }
                } else if (armTransistionStage == 2) {
                    slidesRetract = true;
                    waitSlidesTransition = false;
                    if (armAtPosition()) {
                        armTransistionStage = 3;
                    }
                } else {
                    slidesRetract = false;
                    armReachedPosition = true;
                }
            } else {
                if (teleopModeStart) waitSlidesTransition = false;
                if (teleopModeStart) armReachedPosition = true;
                if (teleopModeStart) armTransistionStage = 3;

                slidesRetract = false;
            }
        } else if (currentMode == TeleopMode.TOUCH_POLE) {
            setTargetSlidesPosition(params.SLIDES_TOUCH_POLE);
            setArmTargetPosition(params.ARM_TOUCH_POLE);

            if(teleopModeStart) waitSlidesTransition = true;
            if(teleopModeStart) armReachedPosition = false;
            if(teleopModeStart) armTransistionStage = 1;

            if(armTransistionStage == 1) {
                slidesRetract = true;
                if(getSlidesPosition() >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    armTransistionStage = 2;
                }
            } else if(armTransistionStage == 2) {
                slidesRetract = true;
                waitSlidesTransition = false;
                if(armAtPosition()) {
                    armTransistionStage = 3;
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if (currentMode == TeleopMode.SPECIMEN_SCORE) {
            if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                if (teleopModeStart) armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
                if (teleopModeStart) slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
            } else if(params.INTAKE_TYPE == IntakeType.CLAW) {
                if (teleopModeStart) armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_CLAW;
                if (teleopModeStart) slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_MAX_CLAW;
            }

            setTargetSlidesPosition(slidesPosSpecimen);
            setArmTargetPosition(armPosSpecimen);

            if(teleopModeStart) waitSlidesTransition = true;
            if(teleopModeStart) armReachedPosition = false;
            if(teleopModeStart) armTransistionStage = 1;

            if(armTransistionStage == 1) {
                slidesRetract = true;
                if(getSlidesPosition() >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    armTransistionStage = 2;
                }
            } else if(armTransistionStage == 2) {
                slidesRetract = true;
                waitSlidesTransition = false;
                if(armAtPosition()) {
                    armTransistionStage = 3;
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if(currentMode == TeleopMode.DEBUG) {
            slidesRetract = false;
            waitSlidesTransition = false;
            if(teleopModeStart) setSlidesPosition(params.SLIDES_MIN_POS);
        } else if(currentMode == TeleopMode.TOUCH_POLE_AUTO) {
            setTargetSlidesPosition(params.SLIDES_TOUCH_POLE_AUTO);
            setArmTargetPosition(params.ARM_TOUCH_POLE_AUTO);

            if(lastTeleopMode != TeleopMode.IDLE) {
                if(teleopModeStart) waitSlidesTransition = true;
                if(teleopModeStart) armReachedPosition = false;
                if(teleopModeStart) armTransistionStage = 1;

                if (armTransistionStage == 1) {
                    slidesRetract = true;
                    if (getSlidesPosition() >= params.SLIDES_TRANSITION_LEN) {
                        waitSlidesTransition = true;
                    } else {
                        armTransistionStage = 2;
                    }
                } else if (armTransistionStage == 2) {
                    slidesRetract = true;
                    waitSlidesTransition = false;
                    if (armAtPosition()) {
                        armTransistionStage = 3;
                    }
                } else {
                    slidesRetract = false;
                    armReachedPosition = true;
                }
            } else {
                if(teleopModeStart) waitSlidesTransition = false;
                if(teleopModeStart) armReachedPosition = true;
                if(teleopModeStart) armTransistionStage = 3;

                slidesRetract = false;
            }
        }

        setArmPosition();

        if(!slidesRetract && currentMode == TeleopMode.INTAKE) {
            armPID.setP(Kp);
            armPID.setI(1);
        } else {
            armPID.setP(Kp);
            armPID.setI(Ki);
        }

        armPID.setTolerance(.1);
        out = armPID.calculate(getArmPosition());
        robot.armMotor.setVelocity(out);

        opMode.telemetry.addData("armReachedPosition", armReachedPosition);
        opMode.telemetry.addData("arm pid out", out);

        teleopModeStart = false;

//            opMode.telemetry.addData("wait for slides: ", waitForSlides);
    }
}