package org.firstinspires.ftc.teamcode.Test;

import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Elev.ElevSARCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Autonomous(name = "elevtest", group = "#a")
public class ElevTest extends OpModeCommand {

   ElevatorSubsystem elevatorSubsystem;

    @Override
    public void initialize() {


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        register(
                elevatorSubsystem = new ElevatorSubsystem(hardwareMap)
        );

        schedule(
                new ElevSARCMD(elevatorSubsystem).andThen(
                        new ElevToPoseCMD(elevatorSubsystem,elevatorSubsystem.ScorePos)
                )

        );

        schedule(new RunCommand(() -> {
            telemetry.addData("elevPos",elevatorSubsystem.ElevR.getCurrentPosition());
            telemetry.update();
        }));
    }
}
