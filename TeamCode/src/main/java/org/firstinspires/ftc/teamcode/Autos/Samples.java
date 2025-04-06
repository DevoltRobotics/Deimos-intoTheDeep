package org.firstinspires.ftc.teamcode.Autos;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.SamplesStartPose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.parkControlPose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.parkPose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.pickup1Pose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.pickup2Pose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.pickup3Pose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.scorePose;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Claw.ClawCloseCMD;
import org.firstinspires.ftc.teamcode.Commands.Claw.ClawOpenCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevSARCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.ExtendCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.RetractCMD;
import org.firstinspires.ftc.teamcode.Commands.intake.intakeInCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristDownCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristUpCMD;
import org.firstinspires.ftc.teamcode.Config.DrivePos;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "samples", group = "##")
public class Samples extends OpModeCommand {





    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    public void createPaths() {
        scorePreload = new Path(new BezierLine(new Point(SamplesStartPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(SamplesStartPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());



    }

    @Override
    public void initialize() {
        follower.setStartingPose(SamplesStartPose);

        createPaths();


        schedule(
                new ElevSARCMD(elevatorSubsystem) .andThen(

                new ParallelDeadlineGroup(
                pedroSubsystem.followPathCmd(scorePreload),
                new ClawCloseCMD(clawSubsystem),
                new ElevToPoseCMD(elevatorSubsystem,elevatorSubsystem.ScorePos),
                new ArmToPoseCMD(armSubsystem, armSubsystem.ScorePos),
                new wristUpCMD(wristSubsystem)


                )).andThen(
                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(grabPickup1),
                                new ClawOpenCMD(clawSubsystem),
                                new ExtendCMD(extendoSubsystem),
                                new intakeInCMD(intakeSubsystem),
                                new wristDownCMD(wristSubsystem),
                                new ElevToPoseCMD(elevatorSubsystem,elevatorSubsystem.TransferPos),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ArmToPoseCMD(armSubsystem, armSubsystem.TransferPos)
                                        )
                                )
                ).andThen(new ParallelCommandGroup(
                                pedroSubsystem.followPathCmd(scorePickup1),
                                new RetractCMD(extendoSubsystem),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new ParallelCommandGroup(
                                        new ClawCloseCMD(clawSubsystem),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new ElevToPoseCMD(elevatorSubsystem,elevatorSubsystem.ScorePos),
                                                new WaitCommand(800),
                                                new ArmToPoseCMD(armSubsystem, armSubsystem.ScorePos)
                                                )
                                )
                        )

                )).andThen(
                        new ParallelCommandGroup(
                        pedroSubsystem.followPathCmd(grabPickup2)

                        )).andThen(
                        pedroSubsystem.followPathCmd(scorePickup2)
                ).andThen(
                        pedroSubsystem.followPathCmd(grabPickup3)
                ).andThen(
                        pedroSubsystem.followPathCmd(scorePickup3)
                ).andThen(pedroSubsystem.followPathCmd(park))

        );
    }
}


