package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {

    Servo wL,wR;

    public WristSubsystem(HardwareMap hMap){
        wL = hMap.get(Servo.class,"wL");
        wR = hMap.get(Servo.class,"wR");
    }

    public void wristUp(){
        wL.setPosition(0.25);
        wR.setPosition(0.75);
    }

    public void wristDown(){
        wL.setPosition(0.51);
        wR.setPosition(0.49);
    }

    public void wristMiddle(){
        wL.setPosition(0.5);
        wR.setPosition(0.5);
    }


}
