package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ArmToPoseCMD extends CommandBase {

    ArmSubsystem subsystem;

    int targetPos;

    public ArmToPoseCMD(ArmSubsystem subsystem, int targetPos) {
        this.subsystem = subsystem;
        this.targetPos = targetPos;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ticks = targetPos;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
