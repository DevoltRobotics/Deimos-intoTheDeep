package org.firstinspires.ftc.teamcode.Commands.Extendo;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSubsystem;

public class ExtendCMD extends CommandBase {

    ExtendoSubsystem subsystem;

    public ExtendCMD(ExtendoSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.Extend();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
