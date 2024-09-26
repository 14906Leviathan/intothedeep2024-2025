package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Params {
    public static int ARM_BUCKET2_SCORE_DEG = 80;
    public static final int ARM_MAX_POS = 120;
    public static final int ARM_MIN_POS = 0;
    public static final double ARM_ZERO = .195;
    public static final double ARM_TICK_PER_DEG = 360/3.22;
    public static final double SLIDE_GROUND_POS = 13;
    public static final double SLIDES_BUCKET_2_SCORE_LEN = 42.5;
    public static final int ARM_EXTENDED_DEG = 15;
    public static final int SLIDES_TICKS_PER_INCH = 161;
    public static final double SLIDES_MAX_POS = 42.5;
    public static final double SLIDES_MIN_POS = 0;
    public static final double INTAKE_MAX_POS = 10;
    public static final double INTAKE_MIN_POS = 2;
    public static final double ARM_IDLE_DEG = 15;
    public static final double INTAKE_SPEED = 1;
    public static final double INTAKE_HOLD_SPEED = .65;
    public static final double INTAKE_IDLE_SPEED = .5;
    public static final double OUTTAKE_SPEED = 0;

    /* Constructor */
    public Params(){
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}
