package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Config.DrivePos.sampleParkControlPose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.sampleParkPose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.samplePickup1Pose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.samplePickup2Pose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.samplePickup3Pose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.sampleScorePose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.samplesStartPose;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Arm.ArmToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Claw.ClawCloseCMD;
import org.firstinspires.ftc.teamcode.Commands.Compund.ScoreCMD;
import org.firstinspires.ftc.teamcode.Commands.Compund.TransferCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevSARCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.ExtendCMD;
import org.firstinspires.ftc.teamcode.Commands.intake.intakeInCMD;
import org.firstinspires.ftc.teamcode.Commands.intake.intakeKeepCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristDownCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristUpCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;

@Autonomous(name = "samples", group = "##")
public class Samples extends OpModeCommand {


    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    public void createPaths() {
        scorePreload = new Path(new BezierLine(new Point(samplesStartPose), new Point(sampleScorePose)));
        scorePreload.setLinearHeadingInterpolation(samplesStartPose.getHeading(), sampleScorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleScorePose), new Point(samplePickup1Pose)))
                .setTangentHeadingInterpolation()

                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickup1Pose), new Point(sampleScorePose)))
                .setLinearHeadingInterpolation(samplePickup1Pose.getHeading(), sampleScorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleScorePose), new Point(samplePickup2Pose)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickup2Pose), new Point(sampleScorePose)))
                .setLinearHeadingInterpolation(samplePickup2Pose.getHeading(), sampleScorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleScorePose), new Point(samplePickup3Pose)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePickup3Pose), new Point(sampleScorePose)))
                .setLinearHeadingInterpolation(samplePickup3Pose.getHeading(), sampleScorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(sampleScorePose), /* Control Point */ new Point(sampleParkControlPose), new Point(sampleParkPose)));
        park.setLinearHeadingInterpolation(sampleScorePose.getHeading(), sampleParkPose.getHeading());


    }

    @Override
    public void initialize() {
        follower.setStartingPose(samplesStartPose);

        createPaths();


        schedule(
                new ParallelCommandGroup(
                        new ClawCloseCMD(clawSubsystem),
                        new ElevSARCMD(elevatorSubsystem),
                        new wristUpCMD(wristSubsystem)

                )
                        .andThen(

                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(scorePreload),
                                        new ExtendCMD(extendoSubsystem),
                                        new intakeInCMD(intakeSubsystem),
                                        new ScoreCMD(elevatorSubsystem,armSubsystem,clawSubsystem)



                                )).andThen(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> follower.setMaxPower(0.65)),


                                        new ParallelDeadlineGroup(
                                                pedroSubsystem.followPathCmd(grabPickup1),
                                        new wristDownCMD(wristSubsystem),
                                                new ElevToPoseCMD(elevatorSubsystem, elevatorSubsystem.TransferPos),
                                                new ArmToPoseCMD(armSubsystem, armSubsystem.TransferPos)
                                        ),

                                        new InstantCommand(() -> follower.setMaxPower(1))
                                )
                        ).andThen(new ParallelCommandGroup(
                                pedroSubsystem.followPathCmd(scorePickup1),
                                new InstantCommand(()->new intakeKeepCMD(intakeSubsystem)
                                        ),
                                new SequentialCommandGroup(
                                        new TransferCMD(extendoSubsystem,wristSubsystem,clawSubsystem),
                                        new WaitCommand(100),
                                        new ScoreCMD(elevatorSubsystem,armSubsystem,clawSubsystem),
                                        new ExtendCMD(extendoSubsystem)
                                ),
                                new WaitCommand(1200)
                        )).andThen(
                                new SequentialCommandGroup(
                                        new InstantCommand(()-> follower.setMaxPower(0.65)),
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(grabPickup2),
                                        new wristDownCMD(wristSubsystem),
                                        new ElevToPoseCMD(elevatorSubsystem, elevatorSubsystem.TransferPos),
                                        new ArmToPoseCMD(armSubsystem, armSubsystem.TransferPos)
                                ),
                                new InstantCommand(()-> follower.setMaxPower(1))

                                )).andThen(new ParallelCommandGroup(
                                pedroSubsystem.followPathCmd(scorePickup2),
                               new SequentialCommandGroup(
                                  new TransferCMD(extendoSubsystem,wristSubsystem,clawSubsystem),
                                 new WaitCommand(100),
                                 new ScoreCMD(elevatorSubsystem,armSubsystem,clawSubsystem),
                                       new ExtendCMD(extendoSubsystem)
                                  ),
                                      new WaitCommand(2000)

                        )).andThen(
                                new SequentialCommandGroup(
                                new InstantCommand(()-> follower.setMaxPower(0.65)),
                                new ParallelDeadlineGroup(
                                 new WaitCommand(2000),
                                pedroSubsystem.followPathCmd(grabPickup3),
                                        new wristDownCMD(wristSubsystem),
                                        new ElevToPoseCMD(elevatorSubsystem, elevatorSubsystem.TransferPos),
                                        new ArmToPoseCMD(armSubsystem, armSubsystem.TransferPos)

                        ),
                          new InstantCommand(()-> follower.setMaxPower(1))
                                )).andThen(new ParallelCommandGroup(
                                pedroSubsystem.followPathCmd(scorePickup3),
                                new SequentialCommandGroup(
                                        new TransferCMD(extendoSubsystem,wristSubsystem,clawSubsystem),
                                        new WaitCommand(100),
                                        new ScoreCMD(elevatorSubsystem,armSubsystem,clawSubsystem),
                                        new ExtendCMD(extendoSubsystem)
                                ),
                                new WaitCommand(2000)


                        )).andThen(pedroSubsystem.followPathCmd(park))

        );
    }
}


