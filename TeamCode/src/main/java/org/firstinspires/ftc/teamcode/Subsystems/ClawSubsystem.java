package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    Servo claw;

    public ClawSubsystem(HardwareMap hMap){
        claw = hMap.get(Servo.class,"garra");
    }

    public void open(){
        claw.setPosition(0.2);
    }
    public void close(){
        claw.setPosition(0.42);
    }
}
