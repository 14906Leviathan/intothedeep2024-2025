package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Misc.GoBildaPinpointDriver;

public class HWProfile {
    public IMU imu;
    public DcMotorEx slidesMotor = null;
    public DcMotorEx armMotor = null;
    public CRServo intakeServo1 = null;
    public CRServo intakeServo2 = null;
    public DcMotorEx motorLF = null;
    public DcMotorEx motorLR = null;
    public DcMotorEx motorRF = null;
    public DcMotorEx motorRR = null;
    public AnalogInput armEncoder = null;
    public SimpleServo clawServo = null;
    public SimpleServo clawPivotServo = null;
    public SimpleServo poleToucher = null;
    public IMU.Parameters imuParams;
    public GoBildaPinpointDriver pinpoint = null;
    public Rev2mDistanceSensor distanceOne = null;

    /* local OpMode members. */
    public HardwareMap hwMap           =  null;

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean driveMotors, boolean purePursuit) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(imuParams);
        imu.resetYaw();

        if(driveMotors) {
            // Define and Initialize Motors
            motorLF = hwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setPower(0);

            motorLR = hwMap.get(DcMotorEx.class, "motorLR");
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLR.setPower(0);

            motorRF = hwMap.get(DcMotorEx.class, "motorRF");
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setDirection(DcMotor.Direction.FORWARD);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRF.setPower(0);

            motorRR = hwMap.get(DcMotorEx.class, "motorRR");
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setDirection(DcMotor.Direction.FORWARD);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRR.setPower(0);
        }

        slidesMotor = hwMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setDirection(DcMotorEx.Direction.FORWARD);
//        slidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setPower(0);               // set motor power

        armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setPower(0);

        armEncoder = hwMap.get(AnalogInput.class, "armEncoder");

        intakeServo1 = hwMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hwMap.get(CRServo.class, "intakeServo2");

        clawServo = new SimpleServo(hwMap, "clawServo", 0, 180, AngleUnit.DEGREES);
        clawServo.setInverted(false);

        clawPivotServo = new SimpleServo(hwMap, "clawServoPivot", 0, 270, AngleUnit.DEGREES);
        clawPivotServo.setInverted(false);

        poleToucher = new SimpleServo(hwMap, "poleToucher", 0, 180, AngleUnit.DEGREES);
        poleToucher.setInverted(true);

        distanceOne = hwMap.get(Rev2mDistanceSensor.class, "distanceOne");

//        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
    }
}  // end of HWProfile Class