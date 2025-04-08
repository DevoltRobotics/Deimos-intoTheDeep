package org.firstinspires.ftc.teamcode.Commands.Pusher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.PusherSubsystem;

public class SavePusherCMD extends CommandBase {
    PusherSubsystem subsystem;

    public SavePusherCMD(PusherSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.save();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
