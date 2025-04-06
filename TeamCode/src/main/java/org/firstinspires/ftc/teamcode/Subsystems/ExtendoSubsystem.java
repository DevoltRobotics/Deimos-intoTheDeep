package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ExtendoSubsystem extends SubsystemBase {
    public Servo ExtL,ExtR;

    public ExtendoSubsystem(final HardwareMap hMap){
        ExtL = hMap.get(Servo.class, "eL");
        ExtR = hMap.get(Servo.class, "eR");
    }

    public void TargetPos(double L,double R){
        ExtL.setPosition(L);
        ExtR.setPosition(R);
    }

    public void Retract(){
        TargetPos(1,0);
    }

    public void Extend(){
        TargetPos(0.45,0.55);
    }
}
