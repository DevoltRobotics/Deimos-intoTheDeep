package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class intakeOutCMD extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public intakeOutCMD(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.Out();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
