package Autos;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import Config.OpModeCommand;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import Subsystems.PedroSubsystem;

@Autonomous(name = "samples", group = "##")
public class Samples extends OpModeCommand {


    PedroSubsystem pedroSubsystem;



    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    private final Pose scorePose = new Pose(16.37, 127.1, Math.toRadians(315));

    private final Pose pickup1Pose = new Pose(25.47, 125, Math.toRadians(-17));

    private final Pose pickup2Pose = new Pose(26.51, 129.18, Math.toRadians(13));

    private final Pose pickup3Pose = new Pose(34.05, 137.24, Math.toRadians(30));

    private final Pose parkPose = new Pose(66.02, 99.03, Math.toRadians(90));

    private final Pose parkControlPose = new Pose(55.62, 138.8, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    public void createPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);

        createPaths();

        register(pedroSubsystem = new PedroSubsystem(follower));

        schedule(
                pedroSubsystem.followPathCmd(scorePreload).andThen(
                     pedroSubsystem.followPathCmd(grabPickup1)
                ).andThen(
                        pedroSubsystem.followPathCmd(scorePickup1)
                ).andThen(
                        pedroSubsystem.followPathCmd(grabPickup2)
                ).andThen(
                        pedroSubsystem.followPathCmd(scorePickup2)
                ).andThen(
                        pedroSubsystem.followPathCmd(grabPickup3)
                ).andThen(
                        pedroSubsystem.followPathCmd(scorePickup3)
                ).andThen(pedroSubsystem.followPathCmd(park))
        );
    }
}


