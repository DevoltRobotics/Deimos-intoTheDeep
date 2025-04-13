package org.firstinspires.ftc.teamcode.Test;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Pedro.PedroVisionAlignCmd;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristPosCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;

@TeleOp
public class CameraPedroIvanTest extends OpModeCommand {
    @Override
    public void initialize() {
        follower.setStartingPose(new Pose());

        vision.init();
        initImu();

        new wristPosCMD(wristSubsystem, 0.4).schedule();
    }

    @Override
    public void start() {
        pedroSubsystem.turnChassisVision(0.4, imu).schedule();
    }
}
