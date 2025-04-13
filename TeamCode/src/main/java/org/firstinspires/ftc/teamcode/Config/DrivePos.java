package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class DrivePos {

    //Importante: Las variables con cada autonomo comienzan con sample/specimen dependiendo
    //Posiblemente sea mejor separlas por clases

    //Autonomo Samples

    public static Pose samplesStartPose = new Pose(9, 111, Math.toRadians(270));

    public static Pose sampleScorePose = new Pose(18.3, 126.9, Math.toRadians(315));

    public static Pose samplePickup1Pose = new Pose(26.1, 125.6  , Math.toRadians(-17));

    public static Pose samplePickup2Pose = new Pose(28.69, 129.78, Math.toRadians(13));

    public static Pose samplePickup3Pose = new Pose(31.54, 134.43, Math.toRadians(30));

    public static Pose sampleParkPose = new Pose(66.02, 99.03, Math.toRadians(90));

    public static Pose sampleParkControlPose = new Pose(55.62, 138.8, Math.toRadians(90));



    //Autonomo Specimen
    public static Pose specimenStartPose = new Pose(9, 56, Math.toRadians(180));

    public static Pose specimenScorePose = new Pose(40.5, 66, Math.toRadians(180));

    public static Pose specimenGoToSample1 = new Pose(48, 34, Math.toRadians(90));

    public static Pose specimenGoToSample1Control = new Pose(22, 40);

    public static Pose specimenLeaveSample1 = new Pose(28, 28, Math.toRadians(55));

    public static Pose specimenGoToSample2 = new Pose(48, 25, Math.toRadians(90));

    public static Pose specimenLeaveSample2 = new Pose(28, 15, Math.toRadians(55));

    public static Pose specimenGoToSample3 = new Pose(57, 15, Math.toRadians(55));

    public static Pose specimenGoToSample3Control = new Pose(51, 23);

    public static Pose specimenLeaveSample3 = new Pose(11.5, 8, Math.toRadians(0));

    public static Pose specimenGrabSpecimen = new Pose(11.5, 36, Math.toRadians(0));
    public static Pose specimenGrabSpecimenControl1 = new Pose(14, 56);
    public static Pose specimenGrabSpecimenControl2 = new Pose(70, 27);

    public static Pose specimenPark = new Pose(16, 47, Math.toRadians(-135));

    //Teleop pathfinding

    public static Pose scoreControl = new Pose(85.9, 122, Math.toRadians(315));


}


