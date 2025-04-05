package org.firstinspires.ftc.teamcode.Commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;

public class wristUpCMD extends CommandBase {

    WristSubsystem subsystem;

    public wristUpCMD(WristSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.wristUp();
    }



}
