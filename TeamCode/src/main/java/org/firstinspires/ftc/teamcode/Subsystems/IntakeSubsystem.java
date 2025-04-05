package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    CRServo GL,GR;

    public IntakeSubsystem(HardwareMap hMap){
        GL = hMap.get(CRServo.class,"GL");
        GR = hMap.get(CRServo.class,"GR");
    }

    public void Power(double L,double R){
        GL.setPower(L);
        GR.setPower(R);
    }

    public void In(){
        Power(1,-1);
    }

    public void Out(){
        Power(-1,1);
    }

    public void Stop(){
        Power(0,0);
    }



}
