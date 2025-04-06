package org.firstinspires.ftc.teamcode.Commands.Arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ArmToPoseTOp extends CommandBase {

    ArmSubsystem subsystem;
    Gamepad gamepad;

    public ArmToPoseTOp(ArmSubsystem subsystem, Gamepad gamepad){
        this.subsystem = subsystem;
        this.gamepad = gamepad;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.ticks += (gamepad.left_trigger - gamepad.right_trigger) * 3;
    }

}
