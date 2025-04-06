package org.firstinspires.ftc.teamcode.Commands.Elev;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

public class ElevToPoseCMD extends CommandBase {

    ElevatorSubsystem subsystem;
    int target;

    public ElevToPoseCMD(ElevatorSubsystem subsystem, int target){
        this.subsystem = subsystem;
        addRequirements(subsystem);
        this.target = target;
    }

    @Override
    public void initialize(){
        subsystem.ticks = target;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(subsystem.elevController.lastError) < 5;
    }
}
