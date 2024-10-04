package org.firstinspires.ftc.teamcode.Abstracts;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;

public abstract class AutoPath {
    public abstract double getEndX();
    public abstract double getEndY();
    public abstract double getEndHeading();
    public abstract PathBuilder getPath();
}
