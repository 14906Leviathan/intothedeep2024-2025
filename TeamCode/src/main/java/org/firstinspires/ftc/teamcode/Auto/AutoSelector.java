package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Abstracts.AutoProgram;

@Config
@Autonomous (name = "Autonomous Selector", group = "Autonomous", preselectTeleOp = "Main TeleOp")
public class AutoSelector extends LinearOpMode {
    private boolean autoIsSelected = false;
    private AutoProgram autoPrograms[] = {new LeftSimple(), new LeftIntake()};
    private int autoIndex = 0;
    private AutoProgram selectedAuto;

    @Override
    public void runOpMode() {
        while (!autoIsSelected && !opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_left) {
                autoIndex--;
            } else if(gamepad1.dpad_right) {
                autoIndex++;
            }

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

        selectedAuto.init(this);
    }
}
