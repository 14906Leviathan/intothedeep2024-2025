package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Params {
    public static int ARM_BUCKET2_SCORE_DEG = 110; //143
    public static int ARM_BUCKET1_SCORE_DEG = 75; //143
    public static int ARM_TOUCH_POLE = 60; //143
    public static final int ARM_MAX_POS = 120;
    public static final int ARM_MIN_POS = 0;
    public static final double ARM_ZERO = .195;
    public static final double ARM_ABS_TICK_PER_DEG = 360/3.22;
    public static final double ARM_TICK_PER_DEG = 5_281/360;
    public static final double SLIDE_GROUND_POS = 13;
    public static final double SLIDES_BUCKET_2_SCORE_LEN = 41;
    public static final double SLIDES_BUCKET_1_SCORE_LEN = 35;
    public static final double SLIDES_TOUCH_POLE = 25;
    public static final int ARM_EXTENDED_DEG = 15;
    public static final int SLIDES_TICKS_PER_INCH = 1922/43;
    public static final double SLIDES_TRANSITION_LEN = 12;
    public static final double ARM_AT_POS_ERROR_DEG = 4;
    public static final double SLIDES_MAX_POS = 42.5;
    public static final double SLIDES_MIN_POS = 0;
    public static final double INTAKE_MAX_POS = 34;
    public static final double INTAKE_MIN_POS = 2;
    public static final double ARM_IDLE_DEG = 25;
    public static final double ARM_INTAKE_MODE_UP_DEG = 6;
    public static final double SLIDES_ARM_ROTATION_THRESHOLD = 10;
    public static final double INTAKE_SPEED = 0;
    public static final double INTAKE_HOLD_SPEED = .4;
    public static final double INTAKE_IDLE_SPEED = .5;
    public static final double OUTTAKE_SPEED = .9;
    public static final double INTAKE_SLOWMODE_MIN_POS = 2;
    public static final double SLOWMODE_XY_MULT = .3;
    public static final double SLOWMODE_TURN_MULT = .3;
    public static final double SLIDE_MOTOR_POWER = .75;

    /* Constructor */
    public Params(){
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}
