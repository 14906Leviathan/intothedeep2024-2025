package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Params {
    public static double ARM_MOTOR_TICKS_PER_REV = 5_281.1;
    public static double ARM_MOTOR_TICKS_PER_DEG = ARM_MOTOR_TICKS_PER_REV/360;
    public static int ARM_SCORE_POS = 60;
    public static final int ARM_MAX_POS = 90;
    public static final int ARM_MIN_POS = 0;

    /* Constructor */
    public Params(){
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}
