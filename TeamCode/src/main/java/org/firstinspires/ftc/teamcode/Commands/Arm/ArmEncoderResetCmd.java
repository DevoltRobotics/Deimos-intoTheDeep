package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ArmEncoderResetCmd extends CommandBase {

    ArmSubsystem subsystem;

    public ArmEncoderResetCmd(ArmSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.brazoPRelative = -15;
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
