package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ElevadoresTest extends OpMode {

    Hardware hardware = new Hardware();

    Action elevUpdateAction = hardware.elevUpdateAction();

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
            hardware.elevToPosAction(-3000).run(null);
        } else {
            hardware.elevToPosAction(-100).run(null);
        }

        elevUpdateAction.run(null);
    }
}
