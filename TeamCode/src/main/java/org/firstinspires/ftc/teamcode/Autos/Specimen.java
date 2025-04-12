package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenGoToSample1;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenGoToSample1Control;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenGoToSample2;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenGoToSample3;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenGoToSample3Control;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenGrabSpecimen;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenLeaveSample1;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenLeaveSample2;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenLeaveSample3;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenPark;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenScorePose;
import static org.firstinspires.ftc.teamcode.Config.DrivePos.specimenStartPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Claw.ClawOpenCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevSARCMD;
import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Extendo.RetractCMD;
import org.firstinspires.ftc.teamcode.Commands.Pusher.PushCMD;
import org.firstinspires.ftc.teamcode.Commands.Pusher.SavePusherCMD;
import org.firstinspires.ftc.teamcode.Commands.Redentor.RedentorCloseCMD;
import org.firstinspires.ftc.teamcode.Commands.Redentor.RedentorOpenCMD;
import org.firstinspires.ftc.teamcode.Commands.wrist.wristUpCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.PusherSubsystem;

@Autonomous(name = "Specimen", group = "##")
public class Specimen extends OpModeCommand {


    private Path scorePreload, scoreSpecimen, park, grabSpecimen, sample1, sample2, sample3, leaveSample1, leaveSample2, leaveSample3, scoreFromSample3;

    Command autoCommand;

    public void createPaths() {
        scorePreload = new Path(new BezierLine(new Point(specimenStartPose), new Point(specimenScorePose)));
        scorePreload.setConstantHeadingInterpolation(specimenScorePose.getHeading());

        sample1 = new Path(new BezierCurve(new Point(specimenScorePose), new Point(specimenGoToSample1Control), new Point(specimenGoToSample1)));
        sample1.setLinearHeadingInterpolation(specimenScorePose.getHeading(), specimenGoToSample1.getHeading());

        leaveSample1 = new Path(new BezierLine(new Point(specimenGoToSample1), new Point(specimenLeaveSample1)));
        leaveSample1.setLinearHeadingInterpolation(specimenGoToSample1.getHeading(), specimenLeaveSample1.getHeading());

        sample2 = new Path(new BezierLine(new Point(specimenLeaveSample1), new Point(specimenGoToSample2)));
        sample2.setLinearHeadingInterpolation(specimenLeaveSample1.getHeading(), specimenGoToSample2.getHeading());

        leaveSample2 = new Path(new BezierLine(new Point(specimenGoToSample2), new Point(specimenLeaveSample2)));
        leaveSample2.setLinearHeadingInterpolation(specimenGoToSample2.getHeading(), specimenLeaveSample2.getHeading());

        sample3 = new Path(new BezierCurve(new Point(specimenLeaveSample2), new Point(specimenGoToSample3Control), new Point(specimenGoToSample3)));
        sample3.setLinearHeadingInterpolation(specimenLeaveSample2.getHeading(), specimenGoToSample3.getHeading());

        leaveSample3 = new Path(new BezierLine(new Point(specimenGoToSample3), new Point(specimenLeaveSample3)));
        leaveSample3.setLinearHeadingInterpolation(specimenGoToSample3.getHeading(), specimenLeaveSample3.getHeading());

        scoreFromSample3 = new Path(new BezierLine(new Point(specimenLeaveSample3), new Point(specimenScorePose)));
        scoreFromSample3.setLinearHeadingInterpolation(specimenLeaveSample3.getHeading(), specimenScorePose.getHeading());

        grabSpecimen = new Path(new BezierLine(new Point(specimenScorePose), new Point(specimenGrabSpecimen)));
        grabSpecimen.setLinearHeadingInterpolation(specimenScorePose.getHeading(), specimenGrabSpecimen.getHeading());

        scoreSpecimen = new Path(new BezierLine(new Point(specimenGrabSpecimen), new Point(specimenScorePose)));
        scoreSpecimen.setLinearHeadingInterpolation(specimenGrabSpecimen.getHeading(), specimenScorePose.getHeading());

        park = new Path(new BezierLine(new Point(specimenScorePose), new Point(specimenPark)));
        park.setLinearHeadingInterpolation(specimenScorePose.getHeading(), specimenPark.getHeading());

    }

    @Override
    public void initialize() {
        follower.setStartingPose(specimenStartPose);

        createPaths();

        new RedentorCloseCMD(redentorSubsystem).schedule();


        autoCommand =
                new ParallelCommandGroup(
                        new wristUpCMD(wristSubsystem),
                        new RetractCMD(extendoSubsystem),
                        new ElevSARCMD(elevatorSubsystem)
                        )
                        .andThen(
                                new ParallelDeadlineGroup(
                                    pedroSubsystem.followPathCmd(scorePreload),
                                    new ElevToPoseCMD(elevatorSubsystem, 1500)))
                        .andThen(
                                new ParallelCommandGroup(
                                    new ElevToPoseCMD(elevatorSubsystem,990),
                                    new ParallelDeadlineGroup(
                                            new WaitCommand(500),
                                            new RedentorOpenCMD(redentorSubsystem)))
                        )

                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(sample1),
                                        new ElevToPoseCMD(elevatorSubsystem,0)))

                        .andThen(
                                new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(leaveSample1),
                                new PushCMD(pusherSubsystem))
                        )

                        .andThen(
                                new ParallelDeadlineGroup(
                                    pedroSubsystem.followPathCmd(sample2),
                                        new SavePusherCMD(pusherSubsystem)
                                )
                        )

                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(leaveSample2),
                                        new PushCMD(pusherSubsystem))
                        )

                        .andThen(
                                new ParallelDeadlineGroup(
                                    pedroSubsystem.followPathCmd(sample3),
                                        new SavePusherCMD(pusherSubsystem)
                                )
                        )


                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(leaveSample3),
                                        new ElevToPoseCMD(elevatorSubsystem,75)
                                )
                        )

                        .andThen(
                                new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(grabSpecimen),
                                new RedentorCloseCMD(redentorSubsystem)))

                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(scoreSpecimen),
                                        new ElevToPoseCMD(elevatorSubsystem,1500)))

                        .andThen(
                                new ParallelCommandGroup(
                                        new ElevToPoseCMD(elevatorSubsystem,990),
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(1000),
                                                new RedentorOpenCMD(redentorSubsystem))))
                        .andThen(
                                new ParallelDeadlineGroup(
                                        new ElevToPoseCMD(elevatorSubsystem,75)
                                )
                        )

                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(grabSpecimen),
                                        new RedentorCloseCMD(redentorSubsystem)))

                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(scoreSpecimen),
                                        new ElevToPoseCMD(elevatorSubsystem,1500)))

                        .andThen(
                                new ParallelCommandGroup(
                                        new ElevToPoseCMD(elevatorSubsystem,990),
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(1000),
                                                new RedentorOpenCMD(redentorSubsystem))))

                        .andThen(
                                new ParallelDeadlineGroup(
                                        new ElevToPoseCMD(elevatorSubsystem,75)))

                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(grabSpecimen),
                                        new RedentorCloseCMD(redentorSubsystem)))



                        .andThen(
                                new ParallelDeadlineGroup(
                                        pedroSubsystem.followPathCmd(scoreSpecimen),
                                        new ElevToPoseCMD(elevatorSubsystem,1500)))
                        .andThen(
                                new ParallelCommandGroup(
                                        new ElevToPoseCMD(elevatorSubsystem,990),
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(1000),
                                                new RedentorOpenCMD(redentorSubsystem))))

                        .andThen(
                                new ParallelDeadlineGroup(
                                        new ElevToPoseCMD(elevatorSubsystem,0)))
        ;
    }

    @Override
    public void start() {
        autoCommand.schedule();
    }
}