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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8,-62,Math.toRadians(270)))
                // DEJAR PRIMER SPECIMEN

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
                .splineToLinearHeading(new Pose2d(-27,-5, Math.toRadians(180)), Math.toRadians(0))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}