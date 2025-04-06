package org.firstinspires.ftc.teamcode.Commands.Elev;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

public class ElevSARCMD extends CommandBase {
    ElevatorSubsystem subsystem;

    public ElevSARCMD(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void execute() {
        subsystem.SARelev();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
