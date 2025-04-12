package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.wrist.wristPosCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;

@Autonomous
@Config
public class CameraTest extends OpModeCommand {

    public static double offsetX = 0;
    public static double offsetY = 0;

    @Override
    public void initialize() {
        vision.init();
        new wristPosCMD(wristSubsystem, 0.35).schedule();

        new RunCommand(() -> {
            RotatedRect rect = vision.getLastRects();
            Point3 rect3d = vision.to3d();
            Point simplePoint = vision.toSimplePoint(offsetX, offsetY);

            if(rect != null && rect3d != null) {
                telemetry.addData("detection", rect);
                telemetry.addData("3d", "%.2f, %.2f, %.2f", rect3d.x +0.7, rect3d.y, rect3d.z);
                telemetry.addData("simple", simplePoint);
                telemetry.update();
            }
        }).schedule();
    }
}
