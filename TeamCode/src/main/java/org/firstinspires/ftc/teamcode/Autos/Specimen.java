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

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.Elev.ElevToPoseCMD;
import org.firstinspires.ftc.teamcode.Commands.Redentor.RedentorCloseCMD;
import org.firstinspires.ftc.teamcode.Commands.Redentor.RedentorOpenCMD;
import org.firstinspires.ftc.teamcode.Config.OpModeCommand;

@Autonomous(name = "Specimen", group = "##")
public class Specimen extends OpModeCommand {


    private Path scorePreload, scoreSpecimen, park, grabSpecimen, sample1, sample2, sample3, leaveSample1, leaveSample2, leaveSample3, scoreFromSample3;


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

        schedule(

                new RedentorCloseCMD(redentorSubsystem),
                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(scorePreload),
                                new ElevToPoseCMD(elevatorSubsystem, 1500)
                        ),
                new RedentorOpenCMD(redentorSubsystem)
                        .andThen(pedroSubsystem.followPathCmd(sample1))
                        .andThen(pedroSubsystem.followPathCmd(leaveSample1))
                        .andThen(pedroSubsystem.followPathCmd(sample2))
                        .andThen(pedroSubsystem.followPathCmd(leaveSample2))
                        .andThen(pedroSubsystem.followPathCmd(sample3))
                        .andThen(pedroSubsystem.followPathCmd(leaveSample3))
                        .andThen(pedroSubsystem.followPathCmd(scoreFromSample3))
                        .andThen(pedroSubsystem.followPathCmd(grabSpecimen))
                        .andThen(pedroSubsystem.followPathCmd(scoreSpecimen))
                        .andThen(pedroSubsystem.followPathCmd(grabSpecimen))
                        .andThen(pedroSubsystem.followPathCmd(scoreSpecimen))
                        .andThen(pedroSubsystem.followPathCmd(grabSpecimen))
                        .andThen(pedroSubsystem.followPathCmd(scoreSpecimen))
                        .andThen(pedroSubsystem.followPathCmd(park))
        );
    }
}