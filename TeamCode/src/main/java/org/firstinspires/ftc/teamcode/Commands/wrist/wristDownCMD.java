package org.firstinspires.ftc.teamcode.Commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;

public class wristDownCMD extends CommandBase {

WristSubsystem subsystem;
    public wristDownCMD(WristSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        subsystem.wristDown();
    }



}

