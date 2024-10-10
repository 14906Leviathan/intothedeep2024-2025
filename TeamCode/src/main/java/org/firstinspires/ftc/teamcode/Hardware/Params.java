package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Enums.IntakeType;

@Config
public class Params {
    public static double ARM_BUCKET2_SCORE_DEG = 110; //143
    public static double ARM_BUCKET1_SCORE_DEG = 70; //143
    public static double ARM_TOUCH_POLE = 60; //143
    public static double ARM_TOUCH_POLE_AUTO = 132; //143
    public static double ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL = 75; //143
    public static double ARM_SPECIMEN_POLE_2_MIN_TWO_WHEEL = 60; //143
    public static double ARM_SPECIMEN_POLE_2_MAX_CLAW = 90; //143
    public static double ARM_SPECIMEN_POLE_2_MIN_CLAW = ARM_SPECIMEN_POLE_2_MAX_CLAW; //143
    public static double ARM_SPECIMEN_INTAKE_TWO_WHEEL = 43; //143
    public static double ARM_SPECIMEN_INTAKE_CLAW = 18.5; //143
    public static final int ARM_MAX_POS = 120;
    public static final int ARM_MIN_POS = 0;
    public static final double ARM_ZERO = .195;
    public static final double ARM_ABS_TICK_PER_DEG = 360/3.22;
    public static final double ARM_TICK_PER_DEG = 5_281/360;
    public static final double SLIDE_GROUND_POS = 13;
    public static final double SLIDES_BUCKET_2_SCORE_LEN = 34;
    public static final double SLIDES_BUCKET_2_SCORE_LEN_CLAW = 41;
    public static final double SLIDES_BUCKET_1_SCORE_LEN = 35;
    public static final double SLIDES_BUCKET_1_SCORE_LEN_CLAW = 37;
    public static final double SLIDES_TOUCH_POLE = 25;
    public static final double SLIDES_TOUCH_POLE_AUTO = 0;
    public static final double SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL = 25;
    public static final double SLIDES_SPECIMEN_POLE_2_MIN_TWO_WHEEL = 25;
    public static final double SLIDES_SPECIMEN_POLE_2_MAX_CLAW = 15;
    public static final double SLIDES_SPECIMEN_POLE_2_MIN_CLAW = 0;
    public static final int ARM_EXTENDED_DEG = 15;
    public static final int SLIDES_TICKS_PER_INCH = 1922/43;
    public static final double SLIDES_TRANSITION_LEN = 12;
    public static final double ARM_AT_POS_ERROR_DEG = 4;
    public static final double SLIDES_MAX_POS = 42.5;
    public static final double SLIDES_SPECIMEN_INTAKE_TWO_WHEEL = 2;
    public static final double SLIDES_SPECIMEN_INTAKE_CLAW = 17;
    public static final double SLIDES_MIN_POS = 0;
    public static final double INTAKE_MAX_POS = 34;
    public static final double INTAKE_DEF_POS = 15;
    public static final double INTAKE_MIN_POS = 2;
    public static final double ARM_IDLE_DEG = 25;
    public static final double ARM_INTAKE_MODE_UP_DEG = 11;
    public static final double ARM_CLAW_MODE_UP_DEG = 9;
    public static final double SLIDES_ARM_ROTATION_THRESHOLD = 10;
    public static final double INTAKE_SPEED = 1;
    public static final DcMotorSimple.Direction INTAKE1_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction INTAKE2_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction OUTAKE1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction OUTAKE2_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final double INTAKE_HOLD_SPEED = .15;
    public static final double INTAKE_IDLE_SPEED = 0;
    public static final double OUTTAKE_SPEED = 1;
    public static final double INTAKE_SLOWMODE_MIN_POS = 2;
    public static final double SLOWMODE_XY_MULT = .3;
    public static final double SLOWMODE_TURN_MULT = .3;
    public static final double SLIDE_MOTOR_POWER = .75;
    public static final IntakeType INTAKE_TYPE = IntakeType.CLAW;
    public static final double CLAW_VERTICAL_GRAB_ANGLE = 3;
    public static final double CLAW_VERTICAL_DROP_ANGLE = 70;
    public static final double CLAW_INSIDE_GRAB_ANGLE = 30;
    public static final double CLAW_INSIDE_DROP_ANGLE = 5;
    public static final double PIVOT_VERTICAL_ANG = 0;
    public static final double PIVOT_HORIZONTAL_ANG = 90;
    public static final double ARM_ERROR_TOLERANCE = .5;


    /* Constructor */
    public Params(){
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}
