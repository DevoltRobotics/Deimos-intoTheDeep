package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class PusherSubsystem extends SubsystemBase {

    Servo Pusher;

    public PusherSubsystem(HardwareMap hMap){
        Pusher = hMap.get(Servo.class,"cola");
    }

    public void push(){
        Pusher.setPosition(1);
    }

    public void save(){
        Pusher.setPosition(0);
    }

}
