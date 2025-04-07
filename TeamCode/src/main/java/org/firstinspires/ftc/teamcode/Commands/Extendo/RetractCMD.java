package org.firstinspires.ftc.teamcode.Commands.Extendo;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSubsystem;

public class RetractCMD extends CommandBase {

    ExtendoSubsystem subsystem;

    public RetractCMD(ExtendoSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.Retract();
    }


    @Override
    public boolean isFinished(){
        return true;
    }
}
