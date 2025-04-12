package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Pedro.PedroVisionAlignCmd;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristPosCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;

@TeleOp
public class CameraPedroTest extends OpModeCommand {
    @Override
    public void initialize() {
        follower.setStartingPose(new Pose());

        vision.init();
        new wristPosCMD(wristSubsystem, 0.35).schedule();
    }

    @Override
    public void start() {
        new PedroVisionAlignCmd(pedroSubsystem, vision, telemetry).schedule();
    }
}
