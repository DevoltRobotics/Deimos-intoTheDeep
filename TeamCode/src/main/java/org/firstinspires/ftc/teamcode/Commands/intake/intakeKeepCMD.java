package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class intakeKeepCMD extends CommandBase {

    IntakeSubsystem subsystem;

    public intakeKeepCMD(IntakeSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }


    @Override
    public void initialize() {
        subsystem.Stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
