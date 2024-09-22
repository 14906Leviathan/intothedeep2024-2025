package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

@Config
public class ArmSubsystem {

    private HWProfile robot;
    public LinearOpMode opMode;
    public Params params;

    public  static void main(String[] args) {

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

    public void setArmPosition(double deg) {
        deg = MathFunctions.clamp(deg, params.ARM_MIN_POS, params.ARM_MAX_POS);

        robot.armMotor.setPower(1);
        robot.armMotor.setTargetPosition((int) Math.round(params.ARM_MOTOR_TICKS_PER_DEG * deg));
    }
}