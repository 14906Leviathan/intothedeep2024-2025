package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.TeleopMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "Main TeleOp", group = "Test")
public class MainTeleOp extends LinearOpMode {
    private Follower follower;

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
    private boolean g1_dpadUpCooldown = false;
    private TeleopMode teleopMode;
    private double intakePosition = params.INTAKE_MAX_POS; //inches
    private boolean aCooldown = false;
    private boolean firstRun = false;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        robot = new HWProfile();
        robot.init(hardwareMap, true);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        intake = new IntakeSubsystem(robot, this, params);

        leftFront = robot.motorLF;
        leftRear = robot.motorLR;
        rightRear = robot.motorRR;
        rightFront = robot.motorRF;

        follower.startTeleopDrive();
        arm.reset();

        waitForStart();

        while(opModeIsActive()) {

            if(firstRun) {
//                teleopMode = TeleopMode.IDLE;
//                arm.setTeleopMode(teleopMode);
//                arm.idle();
            }

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();

            /* *******************INTAKE******************* */

//            if (gamepad1.a && !aCooldown) {
//                aCooldown = true;
//
//                teleopMode = TeleopMode.INTAKE;
//                arm.setTeleopMode(teleopMode);
//                intakePosition = params.INTAKE_MAX_POS;
//            } else if (!gamepad1.a) {
//                aCooldown = false;
//            }
//
//            if (teleopMode == TeleopMode.INTAKE) {
//                arm.setIntakePosition(intakePosition);
//
//                if (gamepad1.right_trigger > .1) {
//                    intakePosition += 1;
//                } else if (gamepad1.left_trigger > .1) {
//                    intakePosition -= 1;
//                }
//
//                if (gamepad1.a) {
//                    arm.setArmPower(.3);
//                    arm.setArmPosition(45);
//                } else if (gamepad1.b) {
//                    arm.setArmPower(.3);
//                    arm.setArmPosition(15);
//                }
//            }

            /* *******************BUCKET SCORE******************* */

//            if(gamepad1.b) {
//                teleopMode = TeleopMode.BUCKET_SCORE;
//                arm.setTeleopMode(teleopMode);
//                arm.setBucket(2);
//            }

            /* *******************INTAKE******************* */

            if(teleopMode == TeleopMode.INTAKE) {
                if(gamepad1.right_bumper) {
                    intake.intake();
                } else if(gamepad1.left_bumper) {
                    intake.outtake();
                } else {
                    intake.hold();
                }
            }

            arm.setArmPower(.3);

            if(gamepad1.a) {
                arm.setArmPosition(15);
            }

            if(gamepad1.b) {
                arm.setArmPosition(5);
            }

            intakePosition = MathFunctions.clamp(intakePosition, params.INTAKE_MIN_POS, params.INTAKE_MAX_POS);

            telemetry.addData("arm position: ", arm.getArmPosition());
            telemetry.addData("arm pid power: ", arm.out);
            telemetry.addData("intake position: ", arm.getIntakePosition());
            telemetry.update();
        }

        arm.setArmPower(0);
    }
}
