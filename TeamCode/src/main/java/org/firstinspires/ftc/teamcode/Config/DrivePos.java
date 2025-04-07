package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class DrivePos {

    //Importante: Las variables con cada autonomo comienzan con sample/specimen dependiendo
    //Posiblemente sea mejor separlas por clases

    //Autonomo Samples

    public static Pose samplesStartPose = new Pose(9, 111, Math.toRadians(270));

    public static Pose sampleScorePose = new Pose(16.37, 127.1, Math.toRadians(315));

    public static Pose samplePickup1Pose = new Pose(25, 125.85  , Math.toRadians(-17));

    public static Pose samplePickup2Pose = new Pose(26.51, 129.18, Math.toRadians(13));

    public static Pose samplePickup3Pose = new Pose(29.5, 134.8, Math.toRadians(30));

    public static Pose sampleParkPose = new Pose(66.02, 99.03, Math.toRadians(90));

    public static Pose sampleParkControlPose = new Pose(55.62, 138.8, Math.toRadians(90));


    //Autonomo Specimen
    public static Pose specimenStartPose = new Pose(9, 56, Math.toRadians(180));

    public static Pose specimenScorePose = new Pose(38, 66, Math.toRadians(180));

    public static Pose specimenGoToSample1 = new Pose(48, 34, Math.toRadians(90));
    public static Pose specimenGoToSample1Control = new Pose(22, 40);

    public static Pose specimenLeaveSample1 = new Pose(28, 28, Math.toRadians(55));

    public static Pose specimenGoToSample2 = new Pose(48, 24, Math.toRadians(90));

    public static Pose specimenLeaveSample2 = new Pose(28, 15, Math.toRadians(55));

    public static Pose specimenGoToSample3 = new Pose(57, 8, Math.toRadians(0));
    public static Pose specimenGoToSample3Control = new Pose(51, 23);

    public static Pose specimenLeaveSample3 = new Pose(11, 8, Math.toRadians(0));

    public static Pose specimenGrabSpecimen = new Pose(38, 66, Math.toRadians(0));

    public static Pose specimenPark = new Pose(16, 47, Math.toRadians(-135));

}


