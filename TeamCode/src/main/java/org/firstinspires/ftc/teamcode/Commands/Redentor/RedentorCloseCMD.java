package org.firstinspires.ftc.teamcode.Commands.Redentor;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.RedentorSubsystem;

public class RedentorCloseCMD extends CommandBase {
    RedentorSubsystem subsystem;

    public RedentorCloseCMD(RedentorSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.Close();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}

