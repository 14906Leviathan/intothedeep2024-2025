package org.firstinspires.ftc.teamcode.AutoPedro;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Abstracts.AutoProgram;

@Config
@Disabled
@Autonomous (name = "Autonomous Selector Pedro", group = "Autonomous", preselectTeleOp = "Main TeleOp")
public class AutoSelectorPedro extends LinearOpMode {
    private boolean autoIsSelected = false;
    private AutoProgram autoPrograms[] = {new LeftSimplePedro(), new LeftIntakePedro()};
    private int autoIndex = 0;
    private AutoProgram selectedAuto;
    private boolean dpLeftCooldown = false;
    private boolean dpRightCooldown = false;

    @Override
    public void runOpMode() {
        while (!autoIsSelected && !opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_left && !dpLeftCooldown) {
                dpLeftCooldown = true;

                autoIndex--;
            } else if(gamepad1.dpad_right && !dpRightCooldown) {
                dpRightCooldown = true;

                autoIndex++;
            }

            if(!gamepad1.dpad_right) dpRightCooldown = false;
            if(!gamepad1.dpad_left) dpLeftCooldown = false;

            if(autoIndex < 0) autoIndex = autoPrograms.length - 1;
            if(autoIndex > autoPrograms.length - 1) autoIndex = 0;

            selectedAuto = autoPrograms[autoIndex];

            telemetry.addData("Auto Name: ", selectedAuto.getAutoName());
            telemetry.addLine("Dpad left/right to select auto");
            telemetry.addLine("A to finalize selection");
            telemetry.update();

            if(gamepad1.a) {
                autoIsSelected = true;
            }
        }

        if(!isStopRequested()) selectedAuto.init(this);
    }
}
