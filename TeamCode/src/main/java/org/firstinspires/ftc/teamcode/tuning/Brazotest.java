package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
@TeleOp
@Config
public class Brazotest extends OpMode {
    public static double p1_arriba = -50;

    Hardware hardware = new Hardware();

    Action brazoUpdateAction = hardware.brazoUpdateAction();
    Action brazoAct1;
    Action brazoAct2;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    boolean brazo = false;

    @Override
    public void loop() {
        hardware.updateArmPosition();

        if (gamepad1.a) {
            brazo = true;
            brazoAct1 = hardware.brazoToPosOnceAction(0);
        }

        if (gamepad1.b) {
            brazo = false;
            brazoAct2 = hardware.brazoToPosOnceAction(275);
        }

        if (brazo) {
            if(brazoAct1 != null)
                brazoAct1.run(null);
        } else {
            if(brazoAct2 != null)
                brazoAct2.run(null);
        }

        brazoUpdateAction.run(null);

        telemetry.addData("state", brazo);
        telemetry.addData("target", hardware.brazoTargetPos);
        telemetry.addData("pos", hardware.brazoPRelative);
    }
}
