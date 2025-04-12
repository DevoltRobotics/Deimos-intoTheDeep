package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;

@TeleOp
public class CameraTest extends OpModeCommand {
    @Override
    public void initialize() {
        vision.init();

        new RunCommand(() -> {
            RotatedRect rect = vision.getLastRects();
            Point3 rect3d = vision.to3d();

            if(rect != null && rect3d != null) {
                telemetry.addData("detection", rect);
                telemetry.addData("3d", "%.2f, %.2f, %.2f", rect3d.x +0.7, rect3d.y, rect3d.z);
                telemetry.update();
            }
        }).schedule();
    }
}
