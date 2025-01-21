package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.48,-65.35,Math.toRadians(90)))
            /*    // DEJAR PRIMER SPECIMEN

                .waitSeconds(.5)
                .strafeToConstantHeading(new Vector2d(-8,-45))
                .waitSeconds(.8)
                .strafeToConstantHeading(new Vector2d(-8,-38))

                // AGARRAR

                .splineToLinearHeading(new Pose2d(-50.5,-47, Math.toRadians(90)), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-51,-40))


                .waitSeconds(1)


                // DEJAR

                .strafeToLinearHeading(new Vector2d(-52.5,-50.5), Math.toRadians(45))//posicion para poner
                .waitSeconds(2)

                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(-59.5,-49, Math.toRadians(90)), Math.toRadians(180), new TranslationalVelConstraint(20))//agarrar 2do
                .strafeToConstantHeading(new Vector2d(-60,-40),null,new ProfileAccelConstraint(-10,10))

                .waitSeconds(1)



                .strafeToLinearHeading(new Vector2d(-52.5,-50.5), Math.toRadians(45))//posicion para poner
                .waitSeconds(2)

                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(-52.5,-45, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(20))

                .strafeToConstantHeading(new Vector2d(-56,-36.5),null,new ProfileAccelConstraint(-10,10))

                .waitSeconds(1)


                .strafeToLinearHeading(new Vector2d(-52.5,-50.5), Math.toRadians(45))//posicion para poner
                .waitSeconds(2)

                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(-27,-5, Math.toRadians(180)), Math.toRadians(0))*/

                        .splineTo(new Vector2d(3.11, -28.22), Math.toRadians(98.22))
                        .splineTo(new Vector2d(17.49, -32.51), Math.toRadians(-1.85))
                        .splineTo(new Vector2d(48.18, -18.99), Math.toRadians(11.57))
                        .splineTo(new Vector2d(49.47, -25.65), Math.toRadians(258.47))
                        .splineTo(new Vector2d(49.25, -32.08), Math.toRadians(259.79))
                        .splineTo(new Vector2d(49.68, -63.85), Math.toRadians(-88.08))
                        .splineTo(new Vector2d(55.05, -16.85), Math.toRadians(76.03))
                        .splineTo(new Vector2d(59.34, -14.06), Math.toRadians(-88.75))
                        .splineTo(new Vector2d(59.77, -17.49), Math.toRadians(223.03))
                        .splineTo(new Vector2d(60.20, -22.00), Math.toRadians(238.74))
                        .splineTo(new Vector2d(58.69, -29.08), Math.toRadians(248.96))
                        .splineTo(new Vector2d(56.98, -40.24), Math.toRadians(262.75))
                        .splineTo(new Vector2d(59.77, -63.42), Math.toRadians(268.37))
                        .splineTo(new Vector2d(37.66, -49.47), Math.toRadians(147.75))
                        .splineTo(new Vector2d(37.23, -69.64), Math.toRadians(268.78))
                        .splineTo(new Vector2d(11.05, -25.65), Math.toRadians(120.76))
                        .splineTo(new Vector2d(34.44, -68.35), Math.toRadians(-61.29))
                        .splineTo(new Vector2d(8.48, -25.43), Math.toRadians(121.17))
                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}