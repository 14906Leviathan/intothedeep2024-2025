package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

@TeleOp(name = "4: Broken Bot", group = "4")
public class BrokenBot extends LinearOpMode {
    private HWProfile robot;
    private Params params;
    private ArmSubsystem arm;
    private TeleopMode currentMode;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void runOpMode() {
        robot = new HWProfile();
        robot.init(hardwareMap, true, false);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        arm.setArmCustomPosition(45);
        arm.setAutoMode(true);
        arm.useMotionProfile(true);

        while(opModeIsActive()) {
            if(gamepad1.dpad_up) {
                robot.motorLF.setPower(1);
            } else {
                robot.motorLF.setPower(0);
            }

            if(gamepad1.dpad_down) {
                robot.motorRF.setPower(1);
            } else {
                robot.motorRF.setPower(0);
            }

            if(gamepad1.dpad_left) {
                robot.motorRR.setPower(1);
            } else {
                robot.motorRR.setPower(0);
            }

            if(gamepad1.dpad_right) {
                robot.motorLR.setPower(1);
            } else {
                robot.motorLR.setPower(0);
            }

            if(gamepad1.a) {
                currentMode = TeleopMode.CUSTOM_POSITION;
                arm.setTeleopMode(currentMode);
                arm.setArmCustomPosition(45);
            } else if (gamepad1.b) {
                currentMode = TeleopMode.CUSTOM_POSITION;
                arm.setTeleopMode(currentMode);
                arm.setArmCustomPosition(15);
            }

            if(gamepad1.right_bumper) {
                robot.clawServo.turnToAngle(0);
            } else if(gamepad1.left_bumper) {
                robot.clawServo.turnToAngle(70);
            }

            if(gamepad1.x) {
                robot.wristServo.turnToAngle(0);
            } else if(gamepad1.y) {
                robot.wristServo.turnToAngle(90);
            }

            arm.update(opModeIsActive());

            telemetry.addData("VSensor: ", voltageSensor.getVoltage());
            telemetry.addData("motorLR", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorLF", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorRR", robot.motorRR.getCurrentPosition());
            telemetry.addData("motorRF", robot.motorRF.getCurrentPosition());
            telemetry.addData("slides motor pos", robot.slidesMotor.getCurrentPosition());
            telemetry.addData("arm abs encoder", robot.armEncoder.getVoltage());
            telemetry.addData("arm encoder", ((double) robot.armMotor.getCurrentPosition()) / params.ARM_TICK_PER_DEG);
            telemetry.addData("slides extension", robot.slidesMotor.getCurrentPosition() / params.SLIDES_TICKS_PER_INCH);
            telemetry.update();
        }
    }
}
