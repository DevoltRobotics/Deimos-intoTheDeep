package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;

public class CRServoSubsystem extends SubsystemBase {
    private final CRServo crServo;

    public CRServoSubsystem(CRServo crServo) {
        this.crServo = crServo;
    }

    public void setPower(double power) {
        crServo.setPower(power);
    }
}
