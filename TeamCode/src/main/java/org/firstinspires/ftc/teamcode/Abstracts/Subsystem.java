package org.firstinspires.ftc.teamcode.Abstracts;

import org.firstinspires.ftc.teamcode.Enums.TeleopMode;

public abstract class Subsystem {
    public abstract void update(boolean opModeActive);
    public abstract void setTeleopMode(TeleopMode mode);
}
