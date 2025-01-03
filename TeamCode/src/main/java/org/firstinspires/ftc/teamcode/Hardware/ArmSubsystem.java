package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Abstracts.Subsystem;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.KookyMotionProfile.MotionProfile;
import org.firstinspires.ftc.teamcode.Hardware.KookyMotionProfile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.Hardware.KookyMotionProfile.ProfileState;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import java.util.concurrent.TimeUnit;

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
//    public static double Kp = 70;
    public static double Kp = 90;
    public static double KpScore = 50;
    public static double Ki = 0.5;
    public static double Kd = 0.0005;

    public static double Kp_Intake = 160;
    public static double Ki_Intake = 1;
    public static double Kd_Intake = 1;

    public static double KpAutoUp = Kp;
    public static double KpAutoDown = Kp;
    public static double KiAutoUp = Ki;
    public static double KiAutoDown = Ki;
    public static double KdAutoUp = Kd;
    public static double KdAutoDown = Kd;

    public static double SlidesKp = 8;
    public static double SlidesKi = 0;
    public static double SlidesKd = 0.35;
    public static double Kf = 0.005;
    private boolean pedroAuto = false;
    private boolean armGoingDown = false;
    private double out = 0;
    public double slidesOut = 0;
    private double oldTargetPos = 0;
    private boolean armUpPark = false;
    private double slidesPower = 0;
    private int bucketScore = 2;
    private static PIDController armPID = new PIDController(Kp, Ki, Kd);
    private static PIDController slidesPID = new PIDController(SlidesKp, SlidesKi, SlidesKd);
    private boolean intakeDownMode = false;
    private boolean intakePushSample = false;
    public boolean teleopModeStart = false;
    private double intakePos = 0;
    private double slidesTargetPos = 0;
    private boolean slidesRetract = false;
    private boolean waitSlidesTransition = false;
    private double armTargetPos = 0;
    private boolean armReachedPosition = false;
    public int armTransistionStage = 0;
    public boolean intakeSpecimen = false;
    public int intakeUpSpecimen = 0;
    private double armPosSpecimen = params.ARM_SPECIMEN_POLE_2_START;
    private double slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
    private TeleopMode lastTeleopMode = TeleopMode.IDLE;
    private boolean outtakeSlidesRetracted = false;
    private boolean armTipBucketScore = false;
    private static double armPower = 1;
    private boolean autoMode = false;
    public static ProfileConstraints motionProfileConstraints = new ProfileConstraints(1, 1, 1);
    private double climbPos = 0;
    private boolean autoLastSample = false;
    private boolean useMotionProfile = false;
    private AnimationType animationType = AnimationType.NORMAL;
    private double armCustomPos = 0;
    private double slidesCustomPos = 0;
    private MotionProfile motionProfile;
    private ElapsedTime timer = new ElapsedTime();
    private static double voltage = 12;
    private boolean useThread = false;
    private static boolean opModeRunning = true;
    private static int threadCycles = 0;
    private int animationDelay = 0;

    private Thread armPidThread = new Thread(() -> {
        double armPosition = (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_ABS_TICK_PER_DEG;
        double out = 0;

        try {
            while (opMode.opModeIsActive() && !Thread.interrupted() && useThread && opModeRunning) {
                armPosition = (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_ABS_TICK_PER_DEG;

                out = armPID.calculate(armPosition) * armPower * (12.0 / voltage);

                try {
                    robot.armMotor.setVelocity(out);
                } catch (Exception e) {
                    // boo hoo an error threw
                    robot.armMotor.setPower(0);
                }

                threadCycles++;
                Thread.sleep(5);
            }
        } catch (Exception e) {

        }
    });

    public void setParkArmUp(boolean set) {
        armUpPark = set;
    }

    public void setAutoLastSample(boolean autoLastSample) {
        this.autoLastSample = autoLastSample;
    }

    public void setAnimationType(AnimationType animationType) {
        this.animationType = animationType;
        animateTransition();
    }

    public void setIntakePush(boolean set) {
        intakePushSample = set;
    }

    private void animateTransition() {
        if (animationType == AnimationType.NORMAL || animationType == AnimationType.FAST || animationType == AnimationType.ROTATE_SOME_WHILE_RETRACT) {
            if (teleopModeStart) waitSlidesTransition = true;
            if (teleopModeStart) armReachedPosition = false;
            if (teleopModeStart) armTransistionStage = 1;

//            opMode.telemetry.addData("armTransitionStage: ", armTransistionStage);

            opMode.telemetry.addData("timer", timer.time(TimeUnit.MILLISECONDS));
            opMode.telemetry.addData("delay", animationDelay);

            if (timer.time(TimeUnit.MILLISECONDS) <= animationDelay) return;

            if (armTransistionStage == 1) {
                slidesRetract = true;

                double slidesTransitionLen = params.SLIDES_TRANSITION_LEN;

                if (animationType == AnimationType.FAST && autoMode) {
                    slidesTransitionLen = slidesTargetPos;
                }

                if (getSlidesPosition() >= slidesTransitionLen + params.SLIDES_ERROR_TOLERANCE) {
                    waitSlidesTransition = true;
                } else {
                    armTransistionStage = 2;
                }
            } else if (armTransistionStage == 2) {
                slidesRetract = true;
                waitSlidesTransition = false;
                if (animationType == AnimationType.NORMAL) {
                    if (armAtPosition()) {
                        armTransistionStage = 3;
                    }
                } else if (animationType == AnimationType.FAST) {
//                    if(armAtPosition()) {
//                        armTransistionStage = 3;
//                    }
                    if (getArmPosition() >= armTargetPos - params.ARM_FAST_OFFSET && getArmPosition() <= armTargetPos + params.ARM_FAST_OFFSET) {
                        armTransistionStage = 3;
                    }
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if (animationType == AnimationType.NONE) {
            waitSlidesTransition = false;
            armReachedPosition = true;
            armTransistionStage = 3;
            slidesRetract = false;
        }

    }

    public void setClimbPos(double newVal) {
        climbPos = newVal;
    }

    public void setPedroAuto(boolean pedroAuto) {
        this.pedroAuto = pedroAuto;
    }

    public void setAutoMode(boolean newState) {
        autoMode = newState;
    }

    public void useMotionProfile(boolean set) {
        useMotionProfile = set;
    }

    public void setAnimationDelay(int set) {
        animationDelay = set;
    }

    public void setTeleopMode(TeleopMode mode) {
        if (lastTeleopMode != mode) lastTeleopMode = currentMode;
        currentMode = mode;
        teleopModeStart = true;

        timer.reset();
        animationType = AnimationType.NORMAL;
        animationDelay = 0;
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
    public ArmSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams) {
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

        if (useThread) armPidThread.start();

        timer.reset();
        timer.startTime();

        motionProfile = new MotionProfile(0, 0, motionProfileConstraints);
    }   // close RRMechOps constructor Method

    public ArmSubsystem(HWProfile myRobot, OpMode myOpMode, Params myParams) {
        robot = myRobot;
//        opMode = myOpMode;
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

    public void setArmCustomPosition(double deg) {
        armCustomPos = deg;
    }

    public void setSlidesCustomPosition(double slides) {
        slidesCustomPos = slides;
    }

    private void setArmTargetPosition(double deg) {
        if (deg != armTargetPos) {
            armGoingDown = (deg < getArmPosition());
        }

        if (waitSlidesTransition == false) armTargetPos = deg;
    }

    private void setArmPosition() {
        if (waitSlidesTransition == false) {
            armPID.setSetPoint(armTargetPos);
        } else {
            if (animationType == AnimationType.ROTATE_SOME_WHILE_RETRACT) {
                armPID.setSetPoint(params.ARM_ROTATE_SOME_MODE_DEG);
            }
        }

        motionProfile.finalPosition = armTargetPos;
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
        return (double) robot.slidesMotor.getCurrentPosition() / params.SLIDES_TICKS_PER_INCH;
//        return robot.pinpoint.getEncoderX() / params.SLIDES_TICKS_PER_INCH;
    }

    public void setSlidesPower(double newPower) {
        slidesPower = newPower;
    }

    private void setTargetSlidesPosition(double len) {
        slidesTargetPos = len;

        setSlidesPosition(slidesTargetPos);
    }

    public double getSlidesTargetPos() {
        return slidesTargetPos;
    }

    public void retractSlidesOuttake() {
        outtakeSlidesRetracted = true;
    }

    public void extendSlidesOuttake() {
        outtakeSlidesRetracted = false;
    }

    private void setSlidesPosition(double len) {
        len = MathFunctions.clamp(len, params.SLIDES_MIN_POS, params.SLIDES_MAX_POS);

        if (slidesRetract || waitSlidesTransition) {
            if (animationType == AnimationType.NORMAL || !autoMode) {
                len = params.SLIDES_TRANSITION_LEN * params.SLIDES_TICKS_PER_INCH;
            } else if (animationType == AnimationType.FAST) {
                len = len * params.SLIDES_TICKS_PER_INCH;
            }
        } else {
            len = len * params.SLIDES_TICKS_PER_INCH;
        }

        slidesPID.setSetPoint((int) len);
    }

    public void setIntakePosition(double len) {
        intakePos = len;
    }

    public void intakeDownMode() {
        intakeDownMode = true;
    }

    public double setArmPositionSpecimen(double pos) {
        armPosSpecimen = pos;
        return armPosSpecimen;
    }

    public double setSlidesPositionSpecimen(double pos) {
        slidesPosSpecimen = pos;

        return slidesPosSpecimen;
    }

    public void intakeUpMode() {
        intakeDownMode = false;
    }

    public boolean armAtPosition() {
        double armPos = getArmPosition();

        double errorTolerance = params.ARM_ERROR_TOLERANCE;

        return (armPos + errorTolerance > armTargetPos && armPos - errorTolerance < armTargetPos);
//        return armPID.atSetPoint();
    }

    public boolean slidesAtPosition() {
        double slidesPos = getSlidesPosition();

        double errorTolerance = params.SLIDES_ERROR_TOLERANCE;

        return (slidesPos + errorTolerance > slidesTargetPos && slidesPos - errorTolerance < slidesTargetPos);
    }

    public boolean slidesAtPosition(double error) {
        double slidesPos = getSlidesPosition();

        return (slidesPos + error > slidesTargetPos && slidesPos - error < slidesTargetPos);
    }

    public void setBucket(int bucket) {
        bucketScore = bucket;
    }

    public double getIntakePosition() {
        return Math.sqrt((getSlidesPosition() * getSlidesPosition()) - (params.SLIDE_GROUND_POS * params.SLIDE_GROUND_POS));
    }

    public double getArmTargetPosition() {
        return armPID.getSetPoint();
    }

    public void update(boolean opModeActive) {
        opModeRunning = opModeActive;

        if (currentMode == TeleopMode.INTAKE) {
//            setSlidesPower(params.SLIDE_MOTOR_POWER);
            double slidesPos = 0;

            if (!intakeSpecimen) {
                if (!autoMode) {
                    slidesPos = MathFunctions.clamp(intakePos, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);
                } else {
                    slidesPos = MathFunctions.clamp(intakePos, 0, params.INTAKE_MAX_POS);
                }

                slidesPos = Math.sqrt((params.SLIDE_GROUND_POS * params.SLIDE_GROUND_POS) + (slidesPos * slidesPos));

                setTargetSlidesPosition(slidesPos);
                double armDeg = 0;
                if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (!autoMode) {
                        armDeg = map(slidesPos, 3, 36, 17 - 5, 25 - 1.5);

                        if (!slidesAtPosition(2.5)) armDeg += 3;
                    } else {
                        if (!pedroAuto) {
                            armDeg = map(slidesPos, 3, 36, 17 - 7, 25 - 5);
                        } else {
                            if (intakePushSample) {
                                armDeg = 18;
                            } else {
                                if (!autoLastSample) {
                                    armDeg = 13;
                                } else {
                                    armDeg = 10;
                                }
                            }
                        }
                    }
                }
                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE)
                    armDeg = map(slidesPos, 3, 36, 17, 25);
                if (armDeg < 0) armDeg = 0;

                if (!intakeDownMode && params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE)
                    armDeg += params.ARM_INTAKE_MODE_UP_DEG;
                if (!intakeDownMode && params.INTAKE_TYPE == IntakeType.CLAW && !autoMode)
                    armDeg += params.ARM_CLAW_MODE_UP_DEG;
                if (!intakeDownMode && params.INTAKE_TYPE == IntakeType.CLAW && autoMode) {
                    if (!autoLastSample) {
                        armDeg += params.ARM_CLAW_MODE_UP_DEG_AUTO;
                    } else {
                        armDeg += params.ARM_CLAW_MODE_UP_DEG_AUTO;
                    }
                }


                setArmTargetPosition(armDeg);
            } else {
                if (!autoMode) {
                    if (intakeDownMode) {
                        setArmTargetPosition(params.ARM_TELEOP_SPECIMEN_INTAKE);
                    } else {
                        setArmTargetPosition(params.ARM_TELEOP_SPECIMEN_INTAKE + params.ARM_SPECIMEN_INTAKE_OFFSET);
                    }
                    setTargetSlidesPosition(params.SLIDES_TELEOP_SPECIMEN_INTAKE);
                } else {
                    if (intakeDownMode) {
                        setArmTargetPosition(params.ARM_AUTO_SPECIMEN_INTAKE);
                    } else {
                        setArmTargetPosition(params.ARM_AUTO_SPECIMEN_INTAKE + params.ARM_SPECIMEN_INTAKE_OFFSET);
                    }
                    setTargetSlidesPosition(params.SLIDES_AUTO_SPECIMEN_INTAKE);
                }
            }

            animateTransition();
        } else if (currentMode == TeleopMode.IDLE) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(params.ARM_IDLE_DEG);
//            setSlidesPower(params.SLIDE_MOTOR_POWER);

            if (lastTeleopMode != TeleopMode.INTAKE) {
                animateTransition();
            } else {
                setAnimationType(AnimationType.NONE);
            }
        } else if (currentMode == TeleopMode.BUCKET_SCORE) {
//            setSlidesPower(params.SLIDE_MOTOR_POWER_OUTTAKE);

            if (bucketScore == 2) {
                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN);
                    }
                    setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (outtakeSlidesRetracted && armTransistionStage == 3) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        if (armTipBucketScore) {
                            if (autoMode) {
                                setTargetSlidesPosition(params.SLIDES_OUTTAKE_RETRACT_MODE_LEN_AUTO);
                                setArmTargetPosition(100);
                            } else {
                                setTargetSlidesPosition(params.SLIDES_OUTTAKE_RETRACT_MODE_LEN);
                                if (params.USING_WRIST) {
                                    setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG_WRIST);
                                } else {
                                    setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
                                }
                            }
                        } else {
                            if (autoMode) {
                                setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW_AUTO);
                                setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG_AUTO);
                            } else {
                                if (!params.USING_WRIST) {
                                    setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW);
                                    setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
                                } else {
                                    setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN_CLAW_WRIST);
                                    setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG_WRIST);
                                }
                            }
                        }
                    }
                }
            } else if (bucketScore == 1) {
                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN);
                    }
                    setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (outtakeSlidesRetracted) {
                        setTargetSlidesPosition(params.SLIDES_MIN_POS);
                    } else {
                        setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN_CLAW);
                    }
                    setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
                }
            }

            if (getArmPosition() < 90 || !teleopModeStart) {
                if (lastTeleopMode == TeleopMode.INTAKE) {
                    setAnimationType(AnimationType.NORMAL);
                }
                animateTransition();
            } else {
                setAnimationType(AnimationType.NONE);
                animateTransition();
            }
        } else if (currentMode == TeleopMode.CLIMB) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(climbPos);

            animateTransition();
        } else if (currentMode == TeleopMode.SPECIMEN_SCORE) {
            if (teleopModeStart) armPosSpecimen = params.ARM_SPECIMEN_POLE_2_SCORE;
            if (teleopModeStart) {
                if (autoMode) {
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START_AUTO;
                } else {
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_START;
                }
            }

            setTargetSlidesPosition(slidesPosSpecimen);
            setArmTargetPosition(armPosSpecimen);

            animateTransition();
        } else if (currentMode == TeleopMode.CUSTOM_POSITION) {
            animationType = AnimationType.NONE;

            setArmTargetPosition(armCustomPos);
            setSlidesCustomPosition(slidesCustomPos);

            animateTransition();
        } else if (currentMode == TeleopMode.TOUCH_POLE_AUTO) {
//            if(teleopModeStart) setParkArmUp(false);

            if (armUpPark) {
                setArmTargetPosition(params.ARM_TOUCH_POLE_AUTO_UP);
            } else {
                setArmTargetPosition(params.ARM_TOUCH_POLE_AUTO_DOWN);
            }

            setTargetSlidesPosition(params.SLIDES_TOUCH_POLE_AUTO);

            setAnimationType(AnimationType.NONE);
        } else if (currentMode == TeleopMode.AUTO_SLIDES_IN) {
            setSlidesPosition(0);
            setSlidesPower(1);
            setArmTargetPosition(123);

            setAnimationType(AnimationType.NONE);
        }

        setArmPosition();

        if (armTransistionStage == 3 && (currentMode == TeleopMode.INTAKE && !intakeSpecimen && intakeDownMode) && !autoMode) {
            armPID.setP(Kp_Intake);
            armPID.setI(Ki_Intake);
            armPID.setD(Kd_Intake);
        } else if (autoMode) {
            if (armGoingDown && !intakeDownMode) {
                armPID.setP(KpAutoDown);
                armPID.setI(KiAutoDown);
                armPID.setD(KdAutoDown);
            } else {
                armPID.setP(KpAutoUp);
                armPID.setI(KiAutoUp);
                armPID.setD(KdAutoUp);
            }
        } else {
            if (currentMode == TeleopMode.BUCKET_SCORE) {
                armPID.setP(KpScore);
            } else {
                armPID.setP(Kp);
            }
            armPID.setI(Ki);
            armPID.setD(Kd);
        }

//        robot.pinpoint.update();

        voltage = robot.voltageSensor.getVoltage();

        armPID.setTolerance(params.ARM_ERROR_TOLERANCE);
        slidesPID.setTolerance(params.SLIDES_ERROR_TOLERANCE);
        slidesPID.setPID(SlidesKp, SlidesKi, SlidesKd);
        out = armPID.calculate(getArmPosition()) * armPower * (12.0 / voltage);
        slidesOut = slidesPID.calculate(robot.slidesMotor.getCurrentPosition()) * slidesPower;

//        opMode.telemetry.addData("mp x: ", pState.x);
//        opMode.telemetry.addData("mp a: ", pState.a);
//        opMode.telemetry.addData("mp v: ", pState.v);
//        opMode.telemetry.addData("arm time: ", (double) timeSinceNewArmSetPos.time(TimeUnit.MILLISECONDS)  / 1000.0);
        opMode.telemetry.addData("arm going down: ", armGoingDown);

        if (!useThread) {
            try {
                robot.armMotor.setVelocity(out);
            } catch (Exception e) {
                // boo hoo an error threw
                robot.armMotor.setPower(0);
            }
        }

        if (opMode.isStopRequested() || !opModeRunning) {
            armPidThread.interrupt();
        }

        robot.slidesMotor.setVelocity(slidesOut);

        teleopModeStart = false;

    }

    public TeleopMode getTeleopMode() {
        return currentMode;
    }

    public boolean getIntakeDownMode() {
        return intakeDownMode;
    }

    public TeleopMode getLastTeleopMode() {
        return lastTeleopMode;
    }

    public int getArmTransistionStage() {
        return armTransistionStage;
    }
}