package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
public class ArmSubsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    private TeleopMode currentMode;
    public static double Kp = 70;
    public static double Kp_Intake = 150;
    public static double Ki = .1;
    public static double Kd = .01;
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

    public  static void main(String[] args) {

    }

    public void setTeleopMode(TeleopMode mode) {
        currentMode = mode;
        teleopModeStart = true;
        armReachedPosition = false;
    }

    /*
     * Constructor method
     */
    public ArmSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;
    }   // close RRMechOps constructor Method

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

    public void intakeUp() {
        intakeGrab = false;
    }

    public boolean armAtPosition() {
        double armPos = getArmPosition();

        return (armPos + params.ARM_AT_POS_ERROR_DEG > armTargetPos && armPos - params.ARM_AT_POS_ERROR_DEG < armTargetPos);
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
            double slidesPos = MathFunctions.clamp(intakePos, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);

            slidesPos = Math.sqrt((params.SLIDE_GROUND_POS*params.SLIDE_GROUND_POS)+(slidesPos*slidesPos));

            setTargetSlidesPosition(slidesPos);
            opMode.telemetry.addData("slidesPosArmClass: ", slidesPos);
            double armDeg = map(slidesPos, 3, 36, 15, 23);
            if(armDeg < 0) armDeg = 0;

            if(!intakeGrab) armDeg += params.ARM_INTAKE_MODE_UP_DEG;

            opMode.telemetry.addData("(DEBUG: From ArmSubsystem) armDeg:", armDeg);

            setArmTargetPosition(armDeg);

            if(getArmPosition() > 35 && !armReachedPosition) {
                slidesRetract = true;
                if(getSlidesPosition() - 5 >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    waitSlidesTransition = false;
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if (currentMode == TeleopMode.IDLE) {
            setTargetSlidesPosition(params.SLIDES_MIN_POS);
            setArmTargetPosition(params.ARM_IDLE_DEG);

            if(!armAtPosition()) {
                slidesRetract = true;
                if(getSlidesPosition() - 5 >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    waitSlidesTransition = false;
                }
            } else {
                slidesRetract = false;
            }
        } else if (currentMode == TeleopMode.BUCKET_SCORE) {
            if(bucketScore == 2) {
                setTargetSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN);
                setArmTargetPosition(params.ARM_BUCKET2_SCORE_DEG);
            } else if(bucketScore == 1) {
                setTargetSlidesPosition(params.SLIDES_BUCKET_1_SCORE_LEN);
                setArmTargetPosition(params.ARM_BUCKET1_SCORE_DEG);
            }

            if(!armAtPosition() && !armReachedPosition) {
                slidesRetract = true;
                if(getSlidesPosition() - 5 >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    waitSlidesTransition = false;
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        } else if (currentMode == TeleopMode.TOUCH_POLE) {
            setTargetSlidesPosition(params.SLIDES_TOUCH_POLE);
            setArmTargetPosition(params.ARM_TOUCH_POLE);

            if(!armAtPosition() && !armReachedPosition) {
                slidesRetract = true;
                if(getSlidesPosition() - 5 >= params.SLIDES_TRANSITION_LEN) {
                    waitSlidesTransition = true;
                } else {
                    waitSlidesTransition = false;
                }
            } else {
                slidesRetract = false;
                armReachedPosition = true;
            }
        }

        setArmPosition();

        if(!slidesRetract && currentMode == TeleopMode.INTAKE) {
            armPID.setP(Kp_Intake);
        } else {
            armPID.setP(Kp);
        }

        out = armPID.calculate(getArmPosition());
        robot.armMotor.setVelocity(out);

        teleopModeStart = false;

//            opMode.telemetry.addData("wait for slides: ", waitForSlides);
    }
}