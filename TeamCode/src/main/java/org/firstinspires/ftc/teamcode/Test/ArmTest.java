package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;

@TeleOp(name = "braoPCos", group = "#a")
public class ArmTest extends OpModeCommand {


    GamepadEx Garra;


    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Garra = new GamepadEx(gamepad2);

        Button brazoUp = new GamepadButton(
            Garra,GamepadKeys.Button.DPAD_UP
        );

        brazoUp.whenPressed(new ArmToPoseCMD(armSubsystem,armSubsystem.ScorePos));

        Button brazoDown = new GamepadButton(
                Garra,GamepadKeys.Button.DPAD_DOWN
        );

        brazoDown.whenPressed(new ArmToPoseCMD(armSubsystem,armSubsystem.TransferPos));

        new RunCommand(()->{
            telemetry.addData("brazo pos",armSubsystem.brazoPRelative);
            telemetry.addData("target",armSubsystem.brazoTargetPos);
            telemetry.update();
        }).schedule();

    }
}
