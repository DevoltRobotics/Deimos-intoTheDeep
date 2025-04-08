package org.firstinspires.ftc.teamcode.Commands.Redentor;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.RedentorSubsystem;

public class RedentorOpenCMD extends CommandBase {
    RedentorSubsystem subsystem;

    public RedentorOpenCMD(RedentorSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.Open();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
