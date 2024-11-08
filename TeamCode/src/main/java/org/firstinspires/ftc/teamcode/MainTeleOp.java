package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "0: Main TeleOp", group = "0")
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private HWProfile robot;
    private Params params;
    private double armPosition = 0;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private boolean g1_dpadDownCooldown = false;
    private boolean rBumperCooldown = false;
    private boolean g1_dpadUpCooldown = false;
    private TeleopMode teleopMode;
    private double intakePosition = params.INTAKE_MAX_POS; //inches
    private boolean aCooldown = false;
    private boolean firstRun = true;
    private boolean LSB_cooldown = false;
    private boolean RSB_cooldown = false;
    private boolean slowMode = false;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private double armPosSpecimen = 0;
    private double slidesPosSpecimen = 0;
    private ElapsedTime loopTime = new ElapsedTime();
    private boolean climbUp = false;
    private double climbPos = 0;
    private MecanumDrive mecDrive;

    @Override
    public void runOpMode() {
        robot = new HWProfile();
        robot.init(hardwareMap, true, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        mecDrive = new MecanumDrive(robot.motorLF, robot.motorRF, robot.motorLR, robot.motorRR);

//        leftFront = robot.motorLF;
//        leftRear = robot.motorLR;
//        rightRear = robot.motorRR;
//        rightFront = robot.motorRF;


        waitForStart();

        robot.slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setAutoMode(false);

        while (opModeIsActive()) {

            loopTime.reset();

            if (firstRun) {
                if (arm.getArmPosition() > 90) {
                    teleopMode = TeleopMode.BUCKET_SCORE;
                } else {
                    teleopMode = TeleopMode.IDLE;
                }
                arm.setTeleopMode(teleopMode);
                arm.update();
                if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    intake.setGrabStyle(GrabStyle.OUTSIDE_GRAB);
                    intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                    intake.outtake();
                    intake.update();
                }

                firstRun = false;
            }

            if (slowMode) {
//                mecDrive.driveFieldCentric(-gamepad1.left_stick_x * params.SLOWMODE_XY_MULT, gamepad1.left_stick_y * params.SLOWMODE_XY_MULT, -gamepad1.right_stick_x * params.SLOWMODE_TURN_MULT, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
                mecDrive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
            } else {
//                mecDrive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, false);
                mecDrive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
            }

            /* *******************INTAKE******************* */


            if (gamepad1.dpad_right) {
                teleopMode = TeleopMode.INTAKE;
                arm.setTeleopMode(teleopMode);
                arm.intakeSpecimen = true;
            }

            if (teleopMode == TeleopMode.INTAKE && arm.intakeSpecimen) {
                grabAngle = GrabAngle.VERTICAL_GRAB;
                grabStyle = GrabStyle.OUTSIDE_GRAB;

                intake.setGrabStyle(grabStyle);
                intake.setGrabAngle(grabAngle);

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    intake.intake();
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.right_bumper && !rBumperCooldown) {
                        intake.toggle();
                        rBumperCooldown = true;
                    }
                }


                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.right_bumper) {
                        arm.intakeUpSpecimen = true;
                    } else {
                        arm.intakeUpSpecimen = false;
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.left_bumper) {
                        arm.intakeUpSpecimen = true;
                    } else {
                        arm.intakeUpSpecimen = false;
                    }
                }
            }


            if (gamepad1.a && !aCooldown) {
                aCooldown = true;

                teleopMode = TeleopMode.INTAKE;
                arm.intakeSpecimen = false;
                arm.setTeleopMode(teleopMode);
                intakePosition = params.INTAKE_DEF_POS;
                if (params.INTAKE_TYPE == IntakeType.CLAW) intake.outtake();
            } else if (!gamepad1.a) {
                aCooldown = false;
            }

            if (teleopMode == TeleopMode.INTAKE && !arm.intakeSpecimen) {
                if (gamepad1.right_trigger > .1) {
                    intakePosition += 3 * gamepad1.right_trigger;
                } else if (gamepad1.left_trigger > .1) {
                    intakePosition -= 3 * gamepad1.left_trigger;
                }

                arm.setIntakePosition(intakePosition);

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.right_bumper) {
                        intake.intake();
                        arm.intakeDownMode();
                    } else if (gamepad1.left_bumper) {
                        intake.outtake();
                        arm.intakeDownMode();
                    } else {
                        intake.hold();
                        arm.intakeUpMode();
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.right_bumper && !rBumperCooldown) {
                        rBumperCooldown = true;
                        intake.toggle();
                    }

                    if (gamepad1.left_bumper) {
                        arm.intakeDownMode();
                    } else {
                        arm.intakeUpMode();
                    }

                    if (gamepad1.left_stick_button && !LSB_cooldown) {
                        LSB_cooldown = true;

                        if (intake.getGrabStyle() == GrabStyle.OUTSIDE_GRAB) {
                            intake.setGrabStyle(GrabStyle.INSIDE_GRAB);
                        } else if (intake.getGrabStyle() == GrabStyle.INSIDE_GRAB) {
                            intake.setGrabStyle(GrabStyle.OUTSIDE_GRAB);
                        }
                    }

                    if (gamepad1.right_stick_button && !RSB_cooldown) {
                        RSB_cooldown = true;

                        if (intake.getGrabAngle() == GrabAngle.VERTICAL_GRAB) {
                            intake.setGrabAngle(GrabAngle.HORIZONTAL_GRAB);
                        } else if (intake.getGrabAngle() == GrabAngle.HORIZONTAL_GRAB) {
                            intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                        }
                    }
                }
            }

            if (teleopMode == TeleopMode.INTAKE) {
                slowMode = true;
            } else {
                slowMode = false;
            }


            /* *******************BUCKET SCORE******************* */

            if (gamepad1.y) {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setTeleopMode(teleopMode);
                arm.setBucket(2);
            }
            if (gamepad1.b) {
                teleopMode = TeleopMode.BUCKET_SCORE;
                arm.setTeleopMode(teleopMode);
                arm.setBucket(1);
            }

            if (teleopMode == TeleopMode.BUCKET_SCORE) {
                grabAngle = grabAngle = GrabAngle.VERTICAL_GRAB;
                intake.setGrabAngle(grabAngle);

                if (gamepad1.right_bumper && !rBumperCooldown) {
                    rBumperCooldown = true;

                    intake.toggle();
                }

                if (gamepad1.left_trigger > .1) {
                    arm.retractSlidesOuttake();
                } else {
                    arm.extendSlidesOuttake();
                }

                arm.setArmTipBucketScore(gamepad1.right_trigger > .1);
            }

            /* *******************IDLE******************* */

            if (gamepad1.x) {
                teleopMode = TeleopMode.IDLE;
                arm.setTeleopMode(teleopMode);
            }

            if (teleopMode == TeleopMode.IDLE) {
                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.right_bumper) {
                        intake.intake();
                    } else if (gamepad1.left_bumper) {
                        intake.outtake();
                    } else {
                        intake.hold();
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.right_bumper && !rBumperCooldown && params.INTAKE_TYPE == IntakeType.CLAW) {
                        rBumperCooldown = true;
                        intake.toggle();
                    }
                }
            }

            /* *******************CLIMB******************* */

            if (gamepad1.dpad_up) {
                teleopMode = TeleopMode.CLIMB;
                arm.setTeleopMode(teleopMode);
                climbUp = true;

                climbPos = params.ARM_CLIMB_UP_MAX_POS;
            }

            if (teleopMode == TeleopMode.CLIMB) {
                arm.poleToucherIn();

                if (gamepad1.dpad_down) {
                    climbUp = false;
                    climbPos = params.ARM_CLIMB_DOWN_POS;
                }

                if (climbUp) {
                    if (gamepad1.left_trigger > .1) {
                        climbPos -= 1;
                    } else if (gamepad1.right_trigger > .1) {
                        climbPos += 1;
                    }

                    climbPos = MathFunctions.clamp(climbPos, params.ARM_CLIMB_UP_MIN_POS, params.ARM_CLIMB_UP_MAX_POS);
                }

                arm.setClimbPos(climbPos);
            } else {
                arm.poleToucherOut();
            }

            /* *******************SPECIMEN SCORE******************* */

            if (gamepad1.dpad_left) {
                teleopMode = TeleopMode.SPECIMEN_SCORE;
                arm.setTeleopMode(teleopMode);
                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE)
                    armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
                if (params.INTAKE_TYPE == IntakeType.CLAW)
                    armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_CLAW_DEFAULT;

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE)
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_MAX_TWO_WHEEL;
                if (params.INTAKE_TYPE == IntakeType.CLAW)
                    slidesPosSpecimen = params.SLIDES_SPECIMEN_POLE_2_DEFAULT_CLAW;
            }

            if (teleopMode == TeleopMode.SPECIMEN_SCORE) {
                if (gamepad1.right_stick_button && !RSB_cooldown) {
                    RSB_cooldown = true;

                    if (intake.getGrabAngle() == GrabAngle.VERTICAL_GRAB) {
                        intake.setGrabAngle(GrabAngle.HORIZONTAL_GRAB);
                    } else if (intake.getGrabAngle() == GrabAngle.HORIZONTAL_GRAB) {
                        intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                    }
                }

                if (gamepad1.right_bumper && !rBumperCooldown && params.INTAKE_TYPE == IntakeType.CLAW) {
                    rBumperCooldown = true;
                    intake.toggle();
                }

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.left_bumper) {
                        intake.idle();
                    } else {
                        intake.intake();
                    }
                }

                if (params.INTAKE_TYPE == IntakeType.TWO_WHEEL_INTAKE) {
                    if (gamepad1.right_trigger > .1) {
                        armPosSpecimen += 2;
                    } else if (gamepad1.left_trigger > .1) {
                        armPosSpecimen -= 2;
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
//                    armPosSpecimen = params.ARM_SPECIMEN_POLE_2_MAX_CLAW;

                    if (gamepad1.right_trigger > .1) {
//                        armPosSpecimen += 7;
                        slidesPosSpecimen += 2.5;
                    } else if (gamepad1.left_trigger > .1) {
//                        armPosSpecimen -= 7;
                        slidesPosSpecimen -= 2.5;
                    }
                }

                armPosSpecimen = arm.setArmPositionSpecimen(armPosSpecimen);
                slidesPosSpecimen = arm.setSlidesPositionSpecimen(slidesPosSpecimen);
            }

            /* *******************RESET IMU******************* */
            if (gamepad1.options) {
                robot.imu.resetYaw();
                params.AUTO_END_HEADING = 0;
            }
            if (!gamepad1.right_bumper) rBumperCooldown = false;


//            arm.setArmPower(.3);

//            if(gamepad1.a) {
//                arm.setArmTargetPosition(90);
//            } else if(gamepad1.b) {
//                arm.setArmTargetPosition(5);
//            }

            if (!gamepad1.left_stick_button) LSB_cooldown = false;
            if (!gamepad1.right_stick_button) RSB_cooldown = false;

            arm.setSlidesPower(params.SLIDE_MOTOR_POWER);
            arm.setArmPower(params.ARM_POWER_DEFAULT);
            arm.update();
            intake.update();

            intakePosition = MathFunctions.clamp(intakePosition, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);

            telemetry.addData("auto end heading: ", params.AUTO_END_HEADING);
            telemetry.addData("slow mode: ", slowMode);
            telemetry.addData("heading: ", robot.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("arm transistion: ", arm.armTransistionStage);
            telemetry.addData("teleop mode start: ", arm.teleopModeStart);
            telemetry.addData("loop time: ", loopTime.time(TimeUnit.MILLISECONDS));
            telemetry.addData("slides raw pos: ", robot.slidesMotor.getCurrentPosition());
            telemetry.addData("arm abs position: ", arm.getArmPosition());
            telemetry.addData("slides out: ", arm.slidesOut);
            telemetry.addData("arm abs position: ", arm.getArmPosition());
            telemetry.addData("intake position: ", arm.getIntakePosition());
            telemetry.addData("slides target pos: ", robot.slidesMotor.getTargetPosition());
            telemetry.addData("slides pos: ", arm.getSlidesPosition());
            telemetry.addData("arm intake pos: ", arm.getIntakePosition());
            telemetry.addData("arm intake grab: ", arm.intakeGrab);
            telemetry.addData("arm at position: ", arm.armAtPosition());
            telemetry.addData("slides at position: ", arm.slidesAtPosition());
            telemetry.addData("arm target position: ", arm.getArmTargetPosition());
            telemetry.addData("slides current: ", robot.slidesMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}