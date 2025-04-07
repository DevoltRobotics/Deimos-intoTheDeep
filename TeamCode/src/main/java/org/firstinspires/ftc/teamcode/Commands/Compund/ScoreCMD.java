package org.firstinspires.ftc.teamcode.Commands.Compund;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Claw.ClawOpenCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

public class ScoreCMD extends ParallelDeadlineGroup {

    public ScoreCMD(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        super(
                        new WaitCommand(1050),
                        new ElevToPoseCMD(elevatorSubsystem, ElevatorSubsystem.ScorePos),
                        new ArmToPoseCMD(armSubsystem, armSubsystem.ScorePos),
                        new SequentialCommandGroup(
                                new WaitCommand((1000)),
                                new ClawOpenCMD(clawSubsystem)
                        )

        );
    }

}
