package org.firstinspires.ftc.teamcode.Commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;

public class wristPosCMD extends CommandBase {

    WristSubsystem subsystem;
    double pos;

    public wristPosCMD(WristSubsystem subsystem, double pos){
        this.subsystem = subsystem;
        this.pos = pos;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.position(pos);
    }

    @Override
    public boolean isFinished(){
     return true;
    }
}
