package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Enums.TeleopMode;
import org.firstinspires.ftc.teamcode.Hardware.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Hardware.Params;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "Broken Bot", group = "Test")
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
        robot.init(hardwareMap, true);
        params = new Params();
        arm = new ArmSubsystem(robot, this, params);

        waitForStart();

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
                currentMode = TeleopMode.DEBUG;
                arm.setTeleopMode(currentMode);
                arm.setArmTargetPosition(45);
            } else if (gamepad1.b) {
                currentMode = TeleopMode.DEBUG;
                arm.setTeleopMode(currentMode);
                arm.setArmTargetPosition(15);
            }

            if(gamepad1.right_bumper) {
                robot.clawServo.turnToAngle(0);
            } else if(gamepad1.left_bumper) {
                robot.clawServo.turnToAngle(70);
            }

            arm.update();

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
