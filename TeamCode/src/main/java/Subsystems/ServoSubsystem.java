package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystem extends SubsystemBase {
    private final Servo servo;

    public ServoSubsystem(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }
}
