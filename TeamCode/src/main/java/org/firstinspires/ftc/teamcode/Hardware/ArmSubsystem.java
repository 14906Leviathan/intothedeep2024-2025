package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public static double SlidesKp = 8;
    public static double SlidesKi = 0;
    public static double SlidesKd = 0.35;
    public static double Kf = 0.005;
    private double out = 0;
    public double slidesOut = 0;
    private double oldTargetPos = 0;
    private double slidesPower = 0;
    private int bucketScore = 2;
    private PIDController armPID = new PIDController(Kp, Ki, Kd);
    private PIDController slidesPID = new PIDController(SlidesKp, SlidesKi, SlidesKd);
    public boolean intakeGrab = false;
    public boolean teleopModeStart = false;
    private double intakePos = 0;
    private double slidesTargetPos = 0;
    private boolean slidesRetract = false;
    private boolean waitSlidesTransition = false;
    private double armTargetPos = 0;
    private boolean armReachedPosition = false;
    public int armTransistionStage = 0;
    public boolean intakeSpecimen = false;
    public boolean intakeUpSpecimen = false;
    private double armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
    private double slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
    private TeleopMode lastTeleopMode = TeleopMode.IDLE;
    private boolean outtakeSlidesRetracted = false;
    private boolean armTipBucketScore = false;
    private double armPower = 1;
    private boolean autoMode = false;
    private double climbPos = 0;
    private boolean autoLastSample = false;

    public void setAutoLastSample(boolean autoLastSample) {
        this.autoLastSample = autoLastSample;
    }

    private void animateTransition() {
        if(teleopModeStart) waitSlidesTransition = true;
        if(teleopModeStart) armReachedPosition = false;
        if(teleopModeStart) armTransistionStage = 1;

        if(armTransistionStage == 1) {
            slidesRetract = true;
            if(getSlidesPosition() >= params.SLIDES_TRANSITION_LEN + params.SLIDES_ERROR_TOLERANCE) {
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
    }

    private void skipTransition() {
        waitSlidesTransition = false;
        armReachedPosition = true;
        armTransistionStage = 3;
        slidesRetract = false;
    }

    public void setClimbPos(double newVal) {
        climbPos = newVal;
    }

    public void setAutoMode(boolean newState) {
        autoMode = newState;
    }

    public void setTeleopMode(TeleopMode mode) {
        if(lastTeleopMode != mode) lastTeleopMode = currentMode;
        currentMode = mode;
        teleopModeStart = true;

        armTransistionStage = 1;
    }

    public void setArmPower(double newPower) {
        armPower = newPower;
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

    public void resetSlidesPosition() {
//        robot.pinpoint.resetPosAndIMU();
        robot.slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getSlidesPosition() {
        return robot.slidesMotor.getCurrentPosition() / params.SLIDES_TICKS_PER_INCH;
//        return robot.pinpoint.getEncoderX() / params.SLIDES_TICKS_PER_INCH;
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

        if(slidesRetract || waitSlidesTransition) {
            len = params.SLIDES_TRANSITION_LEN * params.SLIDES_TICKS_PER_INCH;
        } else {
            len = len * params.SLIDES_TICKS_PER_INCH;
        }

        slidesPID.setSetPoint((int) len);
    }

    public void setIntakePosition(double len) {
        intakePos = len;
    }

    public void intakeDownMode() {
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

    public void intakeUpMode() {
        intakeGrab = false;
    }

    public boolean armAtPosition() {
        double armPos = getArmPosition();

        return (armPos + params.ARM_ERROR_TOLERANCE > armTargetPos && armPos - params.ARM_ERROR_TOLERANCE < armTargetPos);
//        return armPID.atSetPoint();
    }

    public boolean slidesAtPosition() {
        double slidesPos = getSlidesPosition();

        return (slidesPos + params.SLIDES_ERROR_TOLERANCE > slidesTargetPos && slidesPos - params.SLIDES_ERROR_TOLERANCE < slidesTargetPos);
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
//            setSlidesPower(params.SLIDE_MOTOR_POWER);
            double slidesPos = 0;

            if(!autoMode) slidesPos = MathFunctions.clamp(intakePos, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);
            if(autoMode) slidesPos = MathFunctions.clamp(intakePos, 0, params.INTAKE_MAX_POS);

            slidesPos = Math.sqrt((params.SLIDE_GROUND_POS*params.SLIDE_GROUND_POS)+(slidesPos*slidesPos));

            if(!intakeSpecimen) {
                setTargetSlidesPosition(slidesPos);
                opMode.telemetry.addData("slidesPosArmClass: ", slidesPos);
                double armDeg = 0;
                if(params.INTAKE_TYPE == IntakeType.CLAW) {
                    if(!autoMode) {
                        armDeg = map(slidesPos, 3, 36, 17 - 4, 25 - 2);
                    } else {
                        armDeg = map(slidesPos, 3, 36, 17 - 7, 25 - 5);
                    }
                }
                if(params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) armDeg = map(slidesPos, 3, 36, 17, 25);
                if (armDeg < 0) armDeg = 0;

                if (!intakeGrab && params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) armDeg += params.ARM_INTAKE_MODE_UP_DEG;
                if (!intakeGrab && params.INTAKE_TYPE == IntakeType.CLAW && !autoMode) armDeg += params.ARM_CLAW_MODE_UP_DEG;
                if (!intakeGrab && params.INTAKE_TYPE == IntakeType.CLAW && autoMode) {
                    if(!autoLastSample) {
                        armDeg += params.ARM_CLAW_MODE_UP_DEG_AUTO;
                    } else {
                        armDeg += params.ARM_CLAW_MODE_UP_DEG_AUTO_LAST_SAMPLE;
                    }
                }

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
                    if (!intakeUpSpecimen) {
                        setArmTargetPosition(params.ARM_SPECIMEN_INTAKE_CLAW);
                    } else {
                        setArmTargetPosition(params.ARM_SPECIMEN_INTAKE_CLAW + 20);
                    }
                }
            }

            animateTransition();
        } else if (currentMode == TeleopMode.IDLE) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(params.ARM_IDLE_DEG);
//            setSlidesPower(params.SLIDE_MOTOR_POWER);

            if(lastTeleopMode != TeleopMode.INTAKE) {
                animateTransition();
            } else {
                skipTransition();
            }
        } else if (currentMode == TeleopMode.BUCKET_SCORE) {
//            setSlidesPower(params.SLIDE_MOTOR_POWER_OUTTAKE);

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
                            setTargetSlidesPosition(params.SLIDES_OUTTAKE_RETRACT_MODE_LEN);
                        } else {
                            setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW);
                        }
                    }
                        setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
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

            if(getArmPosition() < 90 || !teleopModeStart) {
                animateTransition();
            } else {
                skipTransition();
            }
        } else if (currentMode == TeleopMode.CLIMB) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(climbPos);

            animateTransition();
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

            animateTransition();
        } else if(currentMode == TeleopMode.DEBUG) {
            slidesRetract = false;
            waitSlidesTransition = false;
            if(teleopModeStart) setSlidesPosition(params.SLIDES_MIN_POS);
        } else if(currentMode == TeleopMode.TOUCH_POLE_AUTO) {
            setTargetSlidesPosition(params.SLIDES_TOUCH_POLE_AUTO);
            setArmTargetPosition(params.ARM_TOUCH_POLE_AUTO);

            if(lastTeleopMode != TeleopMode.IDLE) {
                if (armTransistionStage == 1) {
                    animateTransition();
                }
            } else {
                skipTransition();
            }
        } else if(currentMode == TeleopMode.AUTO_SLIDES_IN) {
            setSlidesPosition(0);
            setSlidesPower(1);
            setArmTargetPosition(123);

            skipTransition();
        }

        setArmPosition();

        if(!slidesRetract && currentMode == TeleopMode.INTAKE) {
            armPID.setP(Kp);
            armPID.setI(1);
        } else {
            armPID.setP(Kp);
            armPID.setI(Ki);
        }

//        robot.pinpoint.update();

        armPID.setTolerance(params.ARM_ERROR_TOLERANCE);
        slidesPID.setTolerance(params.SLIDES_ERROR_TOLERANCE);
        slidesPID.setPID(SlidesKp, SlidesKi, SlidesKd);
        out = armPID.calculate(getArmPosition()) * armPower;
        slidesOut = slidesPID.calculate(robot.slidesMotor.getCurrentPosition()) * slidesPower;

        robot.armMotor.setVelocity(out);
        robot.slidesMotor.setVelocity(slidesOut);

        opMode.telemetry.addData("armReachedPosition", armReachedPosition);
        opMode.telemetry.addData("arm pid out", out);

        teleopModeStart = false;

//            opMode.telemetry.addData("wait for slides: ", waitForSlides);
    }
}