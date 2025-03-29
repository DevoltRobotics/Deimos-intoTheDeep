package Actions;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.ServoSubsystem;

public class ServoAction extends CommandBase {
    private final ServoSubsystem servoSubsystem;
    private final double position;
    private boolean isFinished;

    public ServoAction(ServoSubsystem subsystem, double position) {
        this.servoSubsystem = subsystem;
        this.position = position;
        addRequirements(servoSubsystem);
    }

    @Override
    public void initialize() {
        servoSubsystem.setPosition(position);
        isFinished = true; // Instant command
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
