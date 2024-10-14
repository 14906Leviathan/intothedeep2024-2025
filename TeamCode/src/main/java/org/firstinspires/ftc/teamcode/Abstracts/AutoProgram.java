package org.firstinspires.ftc.teamcode.Abstracts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;

public abstract class AutoProgram {
    public abstract void init(LinearOpMode _opMode);
    public abstract void startAuto();
    public abstract String getAutoName();
}
