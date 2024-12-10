package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class CRservoAction implements Action {
    CRServo CRservo;
    double power;

    public CRservoAction(CRServo CRservo, double power) {
        this.CRservo = CRservo;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        CRservo.setPower(power);
        return false;
    }
}