package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class DrivePos {

    public static Pose SamplesStartPose = new Pose(9, 111, Math.toRadians(270));


    public static Pose scorePose = new Pose(16.37, 127.1, Math.toRadians(315));

    public static Pose pickup1Pose = new Pose(25.47, 125, Math.toRadians(-17));

    public static Pose pickup2Pose = new Pose(26.51, 129.18, Math.toRadians(13));

    public static Pose pickup3Pose = new Pose(34.05, 137.24, Math.toRadians(30));

    public static Pose parkPose = new Pose(66.02, 99.03, Math.toRadians(90));

    public static Pose parkControlPose = new Pose(55.62, 138.8, Math.toRadians(90));
}
