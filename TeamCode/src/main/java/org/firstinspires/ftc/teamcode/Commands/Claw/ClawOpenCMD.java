package org.firstinspires.ftc.teamcode.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

public class ClawOpenCMD extends CommandBase {

    ClawSubsystem subsystem;

    public ClawOpenCMD(ClawSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.open();
    }
}
