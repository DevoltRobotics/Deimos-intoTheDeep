package org.firstinspires.ftc.teamcode.Commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class intakeInCMD extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public intakeInCMD(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.In();
    }



    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.Stop();
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}
