package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.OpModeCommand;

public class RedentorSubsystem extends SubsystemBase {

    Servo Redentor;

    public RedentorSubsystem(HardwareMap hardwareMap){
        Redentor = hardwareMap.get(Servo.class,"redentor");
    }

    public void Open(){
        Redentor.setPosition(0);
    }
    public void Close(){
        Redentor.setPosition(1);
    }

}
