package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
@TeleOp
public class Brazotest extends OpMode {
    Hardware hardware = new Hardware();

    Action brazoUpdateAction = hardware.brazoUpdateAction();

    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    boolean brazo = false;

    @Override
    public void loop() {
        if(gamepad1.a){
            brazo = true;
        }

        if(gamepad1.b) {
            brazo = false;
        }

        if(brazo) {
            hardware.brazoToPosAction(-500).run(null);
        } else {
            hardware.brazoToPosAction(0).run(null);
        }

        brazoUpdateAction.run(null);

        telemetry.addData("target", hardware.brazoTargetPos);
        telemetry.addData("pos", hardware.virtual.getCurrentPosition());
    }
}
