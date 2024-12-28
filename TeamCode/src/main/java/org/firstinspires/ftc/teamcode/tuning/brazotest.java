package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class brazotest extends OpMode {

    Hardware hardware = new Hardware();

    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    boolean elev = false;

    @Override
    public void loop() {
        if(gamepad1.a){
            elev = true;
        }

        if(gamepad1.b) {
            elev = false;
        }

        if(elev) {
            hardware.brazoToPosAction(-5000).run(null);
        } else {
            hardware.brazoToPosAction(0).run(null);
        }

        telemetry.addData("pos", hardware.GH2.getCurrentPosition());
    }
}
