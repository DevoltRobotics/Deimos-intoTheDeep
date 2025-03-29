package Actions;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.CRServoSubsystem;

public class CRServoAction extends CommandBase {
    private final CRServoSubsystem crServoSubsystem;
    private final double power;
    private boolean isFinished;

    public CRServoAction(CRServoSubsystem subsystem, double power) {
        crServoSubsystem = subsystem;
        this.power = power;
        addRequirements(crServoSubsystem);
    }

    @Override
    public void initialize() {
        crServoSubsystem.setPower(power);
        isFinished = true; // Command completes immediately after setting power
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
