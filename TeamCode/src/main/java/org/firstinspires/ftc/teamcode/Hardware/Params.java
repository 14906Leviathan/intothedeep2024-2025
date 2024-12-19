package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

@Config
public class Params {
    public static double ARM_FAST_OFFSET = 5;
    public static double ARM_SCORE_SPECIMEN = 62;
    public static double SLIDES_SCORE_SPECIMEN = 6;
    public static double SLIDES_ENSURE_SCORE_SPECIMEN = 6;
    public static double ARM_BUCKET2_SCORE_DEG = 110; //143
    public static double ARM_BUCKET2_SCORE_DEG_AUTO = 110; //143
    public static double ARM_BUCKET1_SCORE_DEG = 70; //143
    public static double ARM_CLIMB_UP_MAX_POS = 120; //143
    public static double ARM_CLIMB_UP_MIN_POS = 105; //143
    public static double ARM_CLIMB_DOWN_POS = 8; //143
    public static double ARM_TOUCH_POLE_AUTO_DOWN = 58; //143
    public static double ARM_TOUCH_POLE_AUTO_UP = 80; //143
    public static double ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL = 75; //143
    public static double ARM_SPECIMEN_POLE_2_MIN_TWO_WHEEL = 60; //143
    public static double ARM_SPECIMEN_POLE_2_MAX_CLAW = 110; //143
    public static double ARM_SPECIMEN_POLE_2_MAX_CLAW_DEFAULT = 100; //143
    public static double ARM_SPECIMEN_POLE_2_MIN_CLAW = 65; //143
    public static double ARM_SPECIMEN_INTAKE_TWO_WHEEL = 43; //143
    public static double ARM_SPECIMEN_INTAKE_CLAW = 45; //143
    public static final int ARM_MAX_POS = 120;
    public static final int ARM_MIN_POS = 0;
    public static final double ARM_ZERO = .195;
    public static final double ARM_ABS_TICK_PER_DEG = 360/3.22;
    public static final double ARM_TICK_PER_DEG = 5_281/360;
    public static final double SLIDE_GROUND_POS = 13;
    public static final double SLIDES_BUCKET_2_SCORE_LEN = 34;
    public static final double SLIDES_BUCKET_2_SCORE_LEN_CLAW = 41.25;
    public static final double SLIDES_BUCKET_2_SCORE_LEN_CLAW_AUTO = 42.25;
    public static final double SLIDES_BUCKET_1_SCORE_LEN = 35;
    public static final double SLIDES_BUCKET_1_SCORE_LEN_CLAW = 33;
    public static final double SLIDES_TOUCH_POLE_AUTO = 15;
    public static final double SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL = 25;
    public static final double SLIDES_SPECIMEN_POLE_2_MIN_TWO_WHEEL = 25;
    public static final double SLIDES_SPECIMEN_POLE_2_MAX_CLAW = 30;
    public static final double SLIDES_SPECIMEN_POLE_2_DEFAULT_CLAW = 12;
    public static final double SLIDES_SPECIMEN_POLE_2_MIN_CLAW = 0;
    public static final int ARM_EXTENDED_DEG = 15;
    public static final double DISTANCE_ONE_OFFSET = 3;
    public static final double SLIDES_TICKS_PER_INCH = 1922/43;
    public static final double SLIDES_TRANSITION_LEN = 6;
    public static final double SLIDES_TRANSITION_LEN_AUTO = 10;
    public static final double SLIDES_MAX_POS = 43;
    public static final double SLIDES_SPECIMEN_INTAKE_TWO_WHEEL = 2;
    public static final double SLIDES_SPECIMEN_INTAKE_CLAW = 11;
    public static final double SLIDES_MIN_POS = 0;
    public static final double SLIDES_OUTTAKE_RETRACT_MODE_LEN = 33;
    public static final double SLIDES_OUTTAKE_RETRACT_MODE_LEN_AUTO = 32;
    public static final double INTAKE_MAX_POS = 29;
    public static final double INTAKE_DEF_POS = 15;
    public static final double INTAKE_MIN_POS = 2;
    public static final double ARM_IDLE_DEG = 25;
    public static final double ARM_ROTATE_SOME_MODE_DEG = 35;
    public static final double ARM_INTAKE_MODE_UP_DEG = 11;
    public static final double ARM_CLAW_MODE_UP_DEG = 5;
    public static final double ARM_CLAW_MODE_UP_DEG_AUTO = 15;
    public static final double ARM_CLAW_MODE_UP_DEG_AUTO_LAST_SAMPLE = 8;
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
    public static final double SLOWMODE_XY_MULT = .5;
    public static final double SLOWMODE_TURN_MULT = .5;
    public static final double SLIDE_MOTOR_POWER = .75;
    public static final double SLIDE_MOTOR_POWER_SLOW_AUTO = .5;
    public static final double ARM_POWER_SLOW_AUTO = .75;
    public static final double ARM_POWER_DEFAULT = 1;
    public static final double SLIDE_MOTOR_POWER_OUTTAKE = .85;
    public static final IntakeType INTAKE_TYPE = IntakeType.CLAW;
    public static final double CLAW_OUTSIDE_GRAB_ANGLE = 70;
    public static final double CLAW_OUTSIDE_DROP_ANGLE = 1.5;
    public static final double CLAW_OUTSIDE_DROP_ANGLE_SHORT = 25;
    public static final double CLAW_INSIDE_GRAB_ANGLE = 5;
    public static final double CLAW_INSIDE_DROP_ANGLE = 30;
    public static final double POLE_TOUCHER_IN = 0;
    public static final double POLE_TOUCHER_OUT = 90;
    public static final double PIVOT_VERTICAL_ANG = 90;
    public static final double PIVOT_HORIZONTAL_ANG = 195;
    public static final double ARM_ERROR_TOLERANCE = 9;
    public static final double ARM_ERROR_TOLERANCE_AUTO = 4;
    public static final double ARM_AUTO_SPECIMEN_INTAKE = 16;
    public static final double ARM_TELEOP_SPECIMEN_INTAKE = 19;
    public static final double SLIDES_AUTO_SPECIMEN_INTAKE = 12;
    public static final double SLIDES_TELEOP_SPECIMEN_INTAKE = 15;
    public static final double SLIDES_ERROR_TOLERANCE = 7;
    public static final double SLIDES_ERROR_TOLERANCE_AUTO = 4;
    public static final double AUTO_DEFAULT_SPEED = .9;
    public static final double AUTO_INTAKE_SPEED = .45;
    public static final double AUTO_PARK_SPEED = .75;
    public static final double AUTO_OUTTAKE_SPEED = .6;
    public static final double TWO_AUTO_INTAKE_Y1_POS = 18;
    public static final double THREE_AUTO_INTAKE_Y1_POS = 5;
    public static final double PEDRO_AUTO_INTAKE_Y1_POS = 4;
    public static double AUTO_END_HEADING = 0;
    public static TeleopMode TELEOP_START_MODE = TeleopMode.IDLE;
    public static double AUTO_SCORE = 0;


    /* Constructor */
    public Params(){
    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}
