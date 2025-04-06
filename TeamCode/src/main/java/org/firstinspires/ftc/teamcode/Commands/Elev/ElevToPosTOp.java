package org.firstinspires.ftc.teamcode.Commands.Elev;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

public class ElevToPosTOp extends CommandBase {

    ElevatorSubsystem subsystem;
    Gamepad gamepad;

   public  ElevToPosTOp(ElevatorSubsystem subsystem, Gamepad gamepad){
       this.subsystem = subsystem;
       this.gamepad = gamepad;
       addRequirements(subsystem);
   }

    @Override
    public void initialize(){
        subsystem.ticks += -gamepad.left_stick_y * 15;
    }

}
