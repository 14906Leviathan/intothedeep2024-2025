package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.opencv.core.Mat;

@Config
public class ArmSubsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    private TeleopMode currentMode;
    private double armTargetPos = 0;
    private double armPower = 0;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = .5;
    private ElapsedTime pidTimer = new ElapsedTime();
    private double derivative = 0;
    private double integralSum = 0;
    private double error = 0;
    private double lastError = 0;
    public double out = 0;
    private int bucketScore = 2;

    private Thread pidLoop = new Thread(() -> {
        while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
            double encoderPosition = getArmPosition();

            // calculate the error
            error = armTargetPos - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / pidTimer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * pidTimer.seconds());

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            out = MathFunctions.clamp(out, 0, armPower);

            robot.armMotor.setPower(out);

            lastError = error;

            // reset the timer for next time
            pidTimer.reset();
        }
    });

    public  static void main(String[] args) {

    }

    public void setArmPower(double newPower) {
        armPower = newPower;
    }

    public void setTeleopMode(TeleopMode mode) {
        currentMode = mode;
    }

    /*
     * Constructor method
     */
    public ArmSubsystem(HWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

    }   // close RRMechOps constructor Method

    public void reset() {
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getArmPosition() {
        return (robot.armEncoder.getVoltage() - params.ARM_ZERO) * params.ARM_TICK_PER_DEG;
    }

    public void setArmPosition(double deg) {
        pidTimer.reset();
        armTargetPos = deg;

        if(!pidLoop.isAlive()) pidLoop.start();
    }

    private double getSlidesPosition() {
        return robot.slidesMotor.getCurrentPosition() / params.SLIDES_TICKS_PER_INCH;
    }

    private void setSlidesPositionTicks(double len) {
        len = MathFunctions.clamp(len, params.ARM_ZERO, params.ARM_EXTENDED_DEG);

        robot.slidesMotor.setPower(1);
        robot.armMotor.setTargetPosition((int) Math.round(params.SLIDES_TICKS_PER_INCH * len));
    }

    private void setSlidesPosition(double len) {
        len = MathFunctions.clamp(len, params.SLIDES_MIN_POS, params.SLIDES_MAX_POS);

        robot.slidesMotor.setPower(1);
        robot.armMotor.setTargetPosition((int) Math.round(len));
    }

    public void idle() {
        setSlidesPosition(params.SLIDES_MIN_POS);
        setArmPosition(params.ARM_IDLE_DEG);
    }

    public void setIntakePosition(double len) {
        len = MathFunctions.clamp(len, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);

        double slidesPos = Math.sqrt((params.SLIDE_GROUND_POS*params.SLIDE_GROUND_POS)+(len*len));

        setSlidesPosition(slidesPos);
    }

    public void updateBucketScorePos() {
        if(bucketScore == 2) {
            setSlidesPosition(params.SLIDES_BUCKET_2_SCORE_LEN);
            setArmPosition(params.ARM_BUCKET2_SCORE_DEG);
        }
    }

    public void setBucket(int bucket) {
        bucketScore = bucket;
        updateBucketScorePos();
    }

    public double getIntakePosition() {
        return Math.sqrt((getSlidesPosition()*getSlidesPosition()) - (params.SLIDE_GROUND_POS*params.SLIDE_GROUND_POS));
    }
}