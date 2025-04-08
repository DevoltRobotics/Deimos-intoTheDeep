package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.Config.DrivePos.sampleScorePose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.scoreControl;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseTOp;
import org.firstinspires.ftc.teamcode.Commands.Claw.ClawOpenCMD;
import org.firstinspires.ftc.teamcode.Commands.Compund.TransferCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPosTOp;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.ExtendCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.RetractCMD;
import org.firstinspires.ftc.teamcode.Commands.intake.intakeInCMD;
import org.firstinspires.ftc.teamcode.Commands.intake.intakeKeepCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristDownCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristUpCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Commands.intake.intakeOutCMD;

@TeleOp(name = "Teleop", group = "##")
public class Teleop extends OpModeCommand {

    GamepadEx Chasis;
    GamepadEx Garra;



    @Override
    public void initialize() {
        Chasis = new GamepadEx(gamepad1);
        Garra = new GamepadEx(gamepad2);
        follower.setStartingPose(PedroSubsystem.EndPose);

        CommandScheduler.getInstance().setDefaultCommand(pedroSubsystem, pedroSubsystem.fieldCentricCmd(gamepad1));
        Button ScorePos = new GamepadButton(
          Chasis, GamepadKeys.Button.DPAD_UP
        );
        ScorePos.whenPressed(new InstantCommand(() -> {
                 PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(follower.getPose()), new Point(sampleScorePose)))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), sampleScorePose.getHeading())
                    .build();

            pedroSubsystem.followPathCmd(path).schedule();
        }));
        Button CancelPath=new  GamepadButton(
                Chasis,GamepadKeys.Button.DPAD_RIGHT
        );

        CancelPath.whenPressed(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(pedroSubsystem));

        }));

        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem,new intakeKeepCMD(intakeSubsystem));
        Button IntakeIn =new GamepadButton(
                Garra,GamepadKeys.Button.A
        );
        IntakeIn.whenHeld(new intakeInCMD(intakeSubsystem));
        Button IntakeOut = new GamepadButton(
                Garra, GamepadKeys.Button.B
        );
        IntakeOut.whenHeld(new intakeOutCMD(intakeSubsystem));
        Button IntakeUp = new GamepadButton(
                Garra, GamepadKeys.Button.Y
        );
        IntakeUp.whenPressed(new wristUpCMD(wristSubsystem));

        Button IntakeDown = new GamepadButton(
                Garra,GamepadKeys.Button.X
        );
        IntakeDown.whenPressed(new wristDownCMD(wristSubsystem));
        Button ClawOpen = new GamepadButton(
                Garra, GamepadKeys.Button.LEFT_BUMPER
        );
        ClawOpen.whenPressed(new ClawOpenCMD(clawSubsystem));
        Button ClawCLosed = new GamepadButton(
                Garra, GamepadKeys.Button.RIGHT_BUMPER
        );
        ClawCLosed.whenPressed(new TransferCMD(extendoSubsystem, wristSubsystem, clawSubsystem));
        Button Extend = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_RIGHT
        );
        Extend.whenPressed(new ExtendCMD(extendoSubsystem));
        Button Retract = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_LEFT
        );
        Retract.whenPressed(new RetractCMD(extendoSubsystem));
        Button ScoreG = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_UP
        );
        ScoreG.whenPressed(new ParallelCommandGroup(
               new ElevToPoseCMD(elevatorSubsystem,elevatorSubsystem.ScorePos),
                new ArmToPoseCMD(armSubsystem, armSubsystem.ScorePos)
        ));
        Button TransferG = new GamepadButton(
                Garra,GamepadKeys.Button.DPAD_DOWN
        );
        TransferG.whenPressed(new ParallelCommandGroup(
                new ElevToPoseCMD(elevatorSubsystem,elevatorSubsystem.TransferPos),
                new ArmToPoseCMD(armSubsystem, armSubsystem.TransferPos)
        ));
        CommandScheduler.getInstance().setDefaultCommand(armSubsystem,new ArmToPoseTOp(armSubsystem, gamepad2));
        CommandScheduler.getInstance().setDefaultCommand(elevatorSubsystem,new ElevToPosTOp(elevatorSubsystem,gamepad2));
        new RunCommand(()->{
            if (Math.abs(gamepad2.right_stick_y)> 0.1){
                CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(elevatorSubsystem));
            }
        });
        new RunCommand(()->{
           if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1){
               CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(armSubsystem));
           }
        });

    }
}
