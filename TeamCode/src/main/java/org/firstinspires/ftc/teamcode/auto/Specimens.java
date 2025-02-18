package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "specimens",group = "#")
public class Specimens extends LinearOpMode {

    PinpointDrive mecanumDrive;
    Hardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(8,-65,Math.toRadians(270));

        mecanumDrive = new PinpointDrive(hardwareMap, startPose);

        hardware = new Hardware();
        hardware.init(hardwareMap);

        hardware.SARelev();

        Action auto = mecanumDrive.actionBuilder(startPose)

                .afterTime(0, hardware.pickS_Action())
                .afterTime(.3,new ParallelAction(
                        hardware.elevToPosAction(-1800),
                        hardware.posicion_inicialAction()
                ))
                .afterTime(1.8,hardware.dropS_Action())
                .strafeToConstantHeading(new Vector2d(-5,-36))
                .waitSeconds(1)


                .strafeToConstantHeading(new Vector2d(28,-40))
                .afterTime(0,hardware.elevToPosAction(-50))
                .strafeToConstantHeading(new Vector2d(28,-10))
                .strafeToConstantHeading(new Vector2d(40,-10))
                .afterTime(0,new ParallelAction(
                        hardware.abrirAction()
                      /*  hardware.ExtendAction()
                        hardware.inclinadoAction(),
                        hardware.elevToPosAction(0),
                        hardware.eskupirAction()*/
                ))
                .strafeToConstantHeading(new Vector2d(40,-53))
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction(),
                        hardware.mantenerAction()
                ))
                .strafeToConstantHeading(new Vector2d(40,-10))
                .afterTime(0,hardware.mantenerAction())
                .strafeToConstantHeading(new Vector2d(52,-10))
                .strafeToConstantHeading(new Vector2d(50.5,-53))
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction(),
                        hardware.mantenerAction()
                ))
                .strafeToConstantHeading(new Vector2d(50.5,-45))
                .strafeToLinearHeading(new Vector2d(25,-60),Math.toRadians(90))
                .afterTime(0,hardware.pickS_Action())
                .afterTime(.2,new ParallelAction(

                        hardware.elevToPosAction(-1800)
                ))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(-4,-35),Math.toRadians(270))
                .afterTime(0,hardware.elevToPosAction(-900))
                .waitSeconds(1)
                .afterTime(0,hardware.dropS_Action())
                .afterTime(.2,hardware.elevToPosAction(-50))
                .strafeToLinearHeading(new Vector2d(25,-60),Math.toRadians(90))
                .afterTime(0,hardware.pickS_Action())
                .build();

        waitForStart();
        Actions.runBlocking(new ParallelAction(
                auto,
                hardware.elevUpdateAction(), // update elevador constantemente
                hardware.brazoUpdateAction()));

    }
}
