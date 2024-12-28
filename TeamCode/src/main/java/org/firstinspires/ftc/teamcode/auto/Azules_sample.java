package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Azules sample", group = "####")
public class Azules_sample extends LinearOpMode {

    PinpointDrive mecanumDrive;
    Hardware hardware;


    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-8, -62, Math.toRadians(270));

        mecanumDrive = new PinpointDrive(hardwareMap, startPose);

        hardware = new Hardware();
        hardware.init(hardwareMap);

        Action auto = mecanumDrive.actionBuilder(startPose)
                // DEJAR PRIMER SPECIMEN
                .afterTime(0, new ParallelAction(
                        hardware.brazoToPosAction(-5100),
                        hardware.cerrarAction()
                ) )
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(-5100),
                        hardware.elevToPosAction(-3900)
                ))
                .afterTime(3, hardware.abrirAction())
                .afterTime(4, new ParallelAction(
                        hardware.brazoToPosAction(0),
                        new SequentialAction(
                                new SleepAction(500), // esperar para empezar a bajar el elevador
                                hardware.elevToPosAction(0)
                        )
                ))
                .strafeToLinearHeading(new Vector2d(-52.5,-50.5), Math.toRadians(45))//posicion para poner

                .waitSeconds(2.2)
                .afterTime(0,new ParallelAction(
                        hardware.abrirAction(),
                        hardware.extendAction(700,1),
                        hardware.brazoToPosAction(0),
                        hardware.inclinadoAction(),
                        hardware.shuparAction()
                ))
                .waitSeconds(0.2)
                // AGARRAR
                .afterTime(0,new ParallelAction(
                        hardware.extendAction(700,1),
                        hardware.inclinadoAction(),
                        hardware.shuparAction(),
                        hardware.brazoToPosAction(0),
                        hardware.elevToPosAction(-100) // ajustar para agarrar
                ))
                .splineToLinearHeading(new Pose2d(-50.5,-49.5, Math.toRadians(90)), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-51,-40),new TranslationalVelConstraint(20),new ProfileAccelConstraint(-10,10))

                .afterTime(0.5, hardware.mantenerAction())
                .afterTime(1.5,hardware.shuparAction())
                .waitSeconds(0.8)


                // DEJAR
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.extendAction(0,1)
                ))
                .afterTime(0.5, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(-5100),
                        hardware.elevToPosAction(-3900)
                ))
                .afterTime(3, hardware.abrirAction())
                .afterTime(4, new ParallelAction(
                        hardware.brazoToPosAction(0),
                        new SequentialAction(
                                new SleepAction(500), // esperar para empezar a bajar el elevador
                                hardware.elevToPosAction(0)
                        )
                ))
                .strafeToLinearHeading(new Vector2d(-52.5,-50.5), Math.toRadians(45))//posicion para poner

                .waitSeconds(2)
                .afterTime(0,new ParallelAction(
                        hardware.abrirAction(),
                        hardware.extendAction(700,1),
                        hardware.brazoToPosAction(0),
                        hardware.inclinadoAction(),
                        hardware.shuparAction()
                ))
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(-59.5,-49.5, Math.toRadians(90)), Math.toRadians(180), new TranslationalVelConstraint(20))//agarrar 2do
                .afterTime(0,hardware.elevToPosAction(-100) )
                .strafeToConstantHeading(new Vector2d(-60,-40),null,new ProfileAccelConstraint(-10,10))
                .afterTime(0.5, hardware.mantenerAction())
                .afterTime(1.5,hardware.shuparAction())
                .waitSeconds(0.8)

                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.extendAction(0,1)
                ))
                .afterTime(0.5, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(-5100),
                        hardware.elevToPosAction(-3900)
                ))
                .afterTime(3, hardware.abrirAction())
                .afterTime(4, new ParallelAction(
                        hardware.brazoToPosAction(0),
                        new SequentialAction(
                                new SleepAction(500), // esperar para empezar a bajar el elevador
                                hardware.elevToPosAction(0)
                        )
                ))
                .strafeToLinearHeading(new Vector2d(-53.7,-51.7), Math.toRadians(45))//posicion para poner

                .waitSeconds(2.2)
                .afterTime(0,new ParallelAction(
                        hardware.abrirAction(),
                        hardware.extendAction(700,1),
                        hardware.inclinadoAction(),
                        hardware.shuparAction(),
                        hardware.brazoToPosAction(0)

                ))
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(-52.5,-45, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(20))
                .afterTime(0,hardware.elevToPosAction(-100) )
                .strafeToConstantHeading(new Vector2d(-56,-36.5),null,new ProfileAccelConstraint(-10,10))
                .afterTime(0.5, hardware.mantenerAction())
                .afterTime(1.5,hardware.shuparAction())
                .waitSeconds(0.8)

                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.extendAction(0,1)
                ))
                .afterTime(0.5, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(-5100),
                        hardware.elevToPosAction(-3900)
                ))
                .afterTime(3, hardware.abrirAction())
                .afterTime(4, new ParallelAction(

                        new SequentialAction(
                                new SleepAction(500), // esperar para empezar a bajar el elevador
                                hardware.elevToPosAction(0)
                        )
                ))
                .strafeToLinearHeading(new Vector2d(-53.5,-51.5), Math.toRadians(45))//posicion para poner

                .waitSeconds(2.2)
                .afterTime(0,new ParallelAction(
                        hardware.abrirAction(),
                        hardware.shuparAction()

                ))
                .waitSeconds(0.2)
                .afterTime(0.5, new ParallelAction(
                        hardware.elevToPosAction(0),
                        hardware.brazoToPosAction(-5350)
                ))
                .splineToLinearHeading(new Pose2d(-25,-5, Math.toRadians(180)), Math.toRadians(0),new TranslationalVelConstraint(65),new ProfileAccelConstraint(-60,60))
                .build();

        waitForStart();
        Actions.runBlocking(new ParallelAction(
                auto,
                hardware.elevUpdateAction() // update elevador constantemente
        ));

    }
}
