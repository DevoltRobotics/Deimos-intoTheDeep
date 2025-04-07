package org.firstinspires.ftc.teamcode.Commands.Compund;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Claw.ClawCloseCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.RetractCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristUpCMD;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;

public class TransferCMD extends SequentialCommandGroup {


    public TransferCMD(ExtendoSubsystem extendoSubsystem, WristSubsystem wristSubsystem, ClawSubsystem clawSubsystem) {
        super(
                new ParallelCommandGroup(
                        new wristUpCMD(wristSubsystem),
                        new RetractCMD(extendoSubsystem)
                ),

                new WaitCommand(700),

                new ClawCloseCMD(clawSubsystem)
        );
    }
}
