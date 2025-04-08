package org.firstinspires.ftc.teamcode.Commands.Pusher;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.PusherSubsystem;

public class PushCMD extends CommandBase {
    PusherSubsystem subsystem;

    public PushCMD(PusherSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.push();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
