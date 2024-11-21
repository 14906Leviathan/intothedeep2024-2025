package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Enums.AnimationType;
import org.firstinspires.ftc.teamcode.Enums.GrabAngle;
import org.firstinspires.ftc.teamcode.Enums.GrabStyle;
import org.firstinspires.ftc.teamcode.Enums.IntakeType;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

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
    private boolean telemetryDebug = false;
    private boolean firstRun = true;
    private boolean LSB_cooldown = false;
    private boolean RSB_cooldown = false;
    private int HBSampleCount = 3;
    private int HighSpecCount = 1;
    private int LBSampleCount = 0;
    private boolean g2DpadUpCooldown = false;
    private boolean g2DpadLeftCooldown = false;
    private boolean g2DpadRightCooldown = false;
    private boolean g2DpadDownCooldown = false;
    private boolean g2RBCooldown = false;
    private boolean g2LBCooldown = false;
    private boolean g2DpadLT = false;
    private boolean g2DpadRT = false;
    private boolean slowMode = false;
    private GrabStyle grabStyle = GrabStyle.OUTSIDE_GRAB;
    private GrabAngle grabAngle = GrabAngle.VERTICAL_GRAB;
    private double armPosSpecimen = 0;
    private double slidesPosSpecimen = 0;
    private ElapsedTime loopTime = new ElapsedTime();
    private boolean climbUp = false;
    private double climbPos = 0;
    private MecanumDrive drive;
    private boolean autoPathingEnabled = false;
    private boolean guideCooldown = false;
    private boolean g2ACooldown = false;
    private PathChain currentPath;
    private boolean runningPath = false;
    private double climbs = 0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerSec = new ElapsedTime();
    private MultipleTelemetry mTelemetry;
    private boolean endgameNotified = false;
    private boolean climbNotified = false;
    private boolean gameOverNotified = false;
//    private MecanumDrive mecDrive;

    @Override
    public void runOpMode() {
        robot = new HWProfile();
        robot.init(hardwareMap, false, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(params.AUTO_END_HEADING)), false);

        leftFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        mecDrive = new MecanumDrive(robot.motorLF, robot.motorRF, robot.motorLR, robot.motorRR);

//        leftFront = robot.motorLF;
//        leftRear = robot.motorLR;
//        rightRear = robot.motorRR;
//        rightFront = robot.motorRF;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        robot.slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setAutoMode(false);

        drive.setUpdateAction(new InstantAction(() -> {
            arm.update();
            intake.update();
        }));

        timer.reset();
        timerSec.reset();

        while (opModeIsActive()) {
            loopTime.reset();
            drive.updatePoseEstimate();

            if (firstRun) {
                if (arm.getArmPosition() > 90) {
                    teleopMode = TeleopMode.BUCKET_SCORE;
                } else {
                    teleopMode = params.TELEOP_START_MODE;
                }
                arm.setTeleopMode(teleopMode);
                arm.update();
                arm.setAnimationType(AnimationType.NONE);
                arm.update();

                if(teleopMode == TeleopMode.TOUCH_POLE_AUTO) {
                    arm.setParkArmUp(true);
                }

                if (teleopMode != TeleopMode.IDLE) {
                    if (teleopMode == TeleopMode.INTAKE) {
                        arm.update();
                        arm.setIntakePosition(params.INTAKE_MIN_POS);
                    }
                }

                arm.update();
                if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    intake.setGrabStyle(GrabStyle.OUTSIDE_GRAB);
                    intake.setGrabAngle(GrabAngle.VERTICAL_GRAB);
                    intake.outtake();
                    intake.update();
                }

                firstRun = false;
            }

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(params.AUTO_END_HEADING);
            double rotX = -gamepad1.left_stick_x * Math.cos(botHeading) - -gamepad1.left_stick_y * Math.sin(botHeading);
            double rotY = -gamepad1.left_stick_x * Math.sin(botHeading) + -gamepad1.left_stick_y * Math.cos(botHeading);

            if (slowMode) {
//                MecanumDrive.driveFieldCentric(-gamepad1.left_stick_x * params.SLOWMODE_XY_MULT, gamepad1.left_stick_y * params.SLOWMODE_XY_MULT, -gamepad1.right_stick_x * params.SLOWMODE_TURN_MULT, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
//                mecDrive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);
//                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * params.SLOWMODE_XY_MULT, -gamepad1.left_stick_x * params.SLOWMODE_XY_MULT, -gamepad1.right_stick_x * params.SLOWMODE_TURN_MULT, false);

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -gamepad1.right_stick_x));
            } else {
//                mecDrive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, false);
//                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
//                mecDrive.driveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, robot.imu.getRobotYawPitchRollAngles().getYaw() + params.AUTO_END_HEADING, true);

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -gamepad1.right_stick_x));
            }

            /* *******************INTAKE******************* */

            if (gamepad1.dpad_right && !autoPathingEnabled) {
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
                        arm.intakeUpSpecimen = 1;
                    } else {
                        arm.intakeUpSpecimen = 0;
                    }
                } else if (params.INTAKE_TYPE == IntakeType.CLAW) {
                    if (gamepad1.left_bumper) {
                        if (intake.closed) {
                            arm.intakeUpSpecimen = 2;
                        } else {
                            arm.intakeUpSpecimen = 1;
                        }
                    } else {
                        arm.intakeUpSpecimen = 0;
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

            if (gamepad1.dpad_left && !autoPathingEnabled) {
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
//                        slidesPosSpecimen += 2.5;
                        armPosSpecimen = 100;
                        slidesPosSpecimen = 12;
                    } else if (gamepad1.left_trigger > .1) {
//                        armPosSpecimen -= 7;
//                        slidesPosSpecimen -= 2.5;
                        armPosSpecimen = 85;
                        slidesPosSpecimen = 6;
                    }
                }

                armPosSpecimen = arm.setArmPositionSpecimen(armPosSpecimen);
                slidesPosSpecimen = arm.setSlidesPositionSpecimen(slidesPosSpecimen);
            }

            /* *******************RESET IMU & POSITION******************* */
            if (gamepad1.options) {
//                follower.resetIMU();
//                robot.imu.resetYaw();
                drive.setPose(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(0)));
                robot.imu.resetYaw();
                params.AUTO_END_HEADING = 0;
                gamepad1.rumble(500);
            }

            if (gamepad1.share) {
                gamepad1.rumble(500);

//                follower.setPose(new Pose(0, 0, follower.getTotalHeading()));
                drive.setPose(new Pose2d(0, 0, drive.pose.heading.toDouble()));
            }

            if (!gamepad1.right_bumper) rBumperCooldown = false;

            /* *******************AUTO PATHING******************* */

            if (gamepad1.guide && !guideCooldown) {
                autoPathingEnabled = !autoPathingEnabled;
                gamepad1.rumbleBlips(2);

                guideCooldown = true;
            }

            if (!gamepad1.guide) guideCooldown = false;

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

            /* *******************NOTIFICATIONS******************* */

            if(timer.time(TimeUnit.SECONDS) >= (120 - 30) && !endgameNotified) {
                endgameNotified = true;
                gamepad1.rumbleBlips(1);
            }

            if(timer.time(TimeUnit.SECONDS) >= (120 - 8) && !climbNotified) {
                climbNotified = true;
                gamepad1.rumble(750);
            }

            if(timer.time(TimeUnit.SECONDS) >= 120 && !gameOverNotified) {
                gameOverNotified = true;
                gamepad1.rumbleBlips(2);
            }


            /* *******************TELEMETRY AND SCORING******************* */

            mTelemetry.addLine("A: Debug Telemetry");
            mTelemetry.addLine("Dpad Up/Down: High Bucket Samples");
            mTelemetry.addLine("Dpad Right/Left: High Pole Specimen");
            mTelemetry.addLine("Right/Left Trigger: Climbs");

            if (gamepad2.a && !g2ACooldown) {
                g2ACooldown = true;
                telemetryDebug = !telemetryDebug;
            }

            if (!gamepad2.a) g2ACooldown = false;

            //Sample
            if(gamepad2.dpad_up && !g2DpadUpCooldown) {
                g2DpadUpCooldown = true;
                HBSampleCount++;
            }
            if(!gamepad2.dpad_up) g2DpadUpCooldown = false;

            if(gamepad2.dpad_down && !g2DpadDownCooldown) {
                g2DpadDownCooldown = true;
                HBSampleCount--;
            }
            if(!gamepad2.dpad_down) g2DpadDownCooldown = false;


            if(gamepad2.right_bumper && !g2RBCooldown) {
                g2RBCooldown = true;
                LBSampleCount++;
            }
            if(!gamepad2.right_bumper) g2RBCooldown = false;

            if(gamepad2.left_bumper && !g2LBCooldown) {
                g2LBCooldown = true;
                LBSampleCount--;
            }
            if(!gamepad2.left_bumper) g2LBCooldown = false;

            //Specimen
            if(gamepad2.dpad_right && !g2DpadRightCooldown) {
                g2DpadRightCooldown = true;
                HighSpecCount++;
            }
            if(!gamepad2.dpad_right) g2DpadRightCooldown = false;

            if(gamepad2.dpad_left && !g2DpadLeftCooldown) {
                g2DpadLeftCooldown = true;
                HighSpecCount--;
            }
            if(!gamepad2.dpad_left) g2DpadLeftCooldown = false;

            //Climb
            if(gamepad2.right_trigger > .1 && !g2DpadRT) {
                g2DpadRT = true;
                climbs++;
            }
            if(gamepad2.right_trigger < .1) g2DpadRT = false;

            if(gamepad2.left_trigger > .1 && !g2DpadLT) {
                g2DpadLT = true;
                climbs--;
            }
            if(gamepad2.left_trigger < .1) g2DpadLT = false;

            climbs = MathFunctions.clamp(climbs, 0, 2);

            double autoScore = 37;

            double timeMin = 1 - timer.time(TimeUnit.MINUTES);
            double timeSec = (60 - timerSec.time(TimeUnit.SECONDS));

            if(timerSec.time(TimeUnit.SECONDS) >= 60) timerSec.reset();

            if(params.AUTO_SCORE != 0) autoScore = params.AUTO_SCORE;

            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addData("score: ", 37 + (HBSampleCount*8) + (LBSampleCount*6) + (HighSpecCount*10) + (climbs*15));
            mTelemetry.addData("time: ", ((int) timeMin + ":" + (int) timeSec));
            mTelemetry.addData("high samples: ", HBSampleCount);
            mTelemetry.addData("high rung samples: ", HighSpecCount);
            mTelemetry.addData("climbs: ", climbs);
            mTelemetry.addData("loop time: ", loopTime.time(TimeUnit.MILLISECONDS));

            if(telemetryDebug) {
                mTelemetry.addData("auto pathing enabled: ", autoPathingEnabled);
                mTelemetry.addData("robot x: ", drive.pose.position.x);
                mTelemetry.addData("robot y: ", drive.pose.position.y);
                mTelemetry.addData("auto end heading: ", params.AUTO_END_HEADING);
                mTelemetry.addData("slow mode: ", slowMode);
                mTelemetry.addData("heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
                mTelemetry.addData("arm transistion: ", arm.armTransistionStage);
                mTelemetry.addData("teleop mode start: ", arm.teleopModeStart);
                mTelemetry.addData("slides raw pos: ", robot.slidesMotor.getCurrentPosition());
                mTelemetry.addData("arm abs position: ", arm.getArmPosition());
                mTelemetry.addData("slides out: ", arm.slidesOut);
                mTelemetry.addData("arm abs position: ", arm.getArmPosition());
                mTelemetry.addData("intake position: ", arm.getIntakePosition());
                mTelemetry.addData("slides target pos: ", robot.slidesMotor.getTargetPosition());
                mTelemetry.addData("slides pos: ", arm.getSlidesPosition());
                mTelemetry.addData("arm intake pos: ", arm.getIntakePosition());
                mTelemetry.addData("arm intake grab: ", arm.intakeGrab);
                mTelemetry.addData("arm at position: ", arm.armAtPosition());
                mTelemetry.addData("slides at position: ", arm.slidesAtPosition());
                mTelemetry.addData("arm target position: ", arm.getArmTargetPosition());
                mTelemetry.addData("slides current: ", robot.slidesMotor.getCurrent(CurrentUnit.AMPS));
            }
            mTelemetry.update();
        }
    }

    public void safeWait(int millis) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (time.time(TimeUnit.MILLISECONDS) >= millis) {
            arm.update();
            intake.update();
        }
    }
}