package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.opencv.core.Mat;

@Autonomous(name = "Sample_izquierdo",group = "###")
public class Samples_izquierda extends LinearOpMode {

    PinpointDrive mecanumDrive;
    Hardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, -61, Math.toRadians(0));

        mecanumDrive = new PinpointDrive(hardwareMap, startPose);

        hardware = new Hardware();
        hardware.init(hardwareMap);

        hardware.brazoTargetPos = 280;
        hardware.SARelev();

        Action auto = mecanumDrive.actionBuilder(startPose)

                .afterTime(0, new ParallelAction(
                        hardware.cerrarAction(),
                        hardware.elevToPosAction(-1650),
                        hardware.posicion_inicialAction()
                ))
                .afterTime(1.5, new ParallelAction(
                        hardware.abrirAction()
                ))
                .afterTime(1.7, hardware.brazoToPosOnceAction(0)
                )
                .strafeToLinearHeading(new Vector2d(-52, -50), Math.toRadians(45))//posicion para poner
                .waitSeconds(.5)
                .afterTime(1, new ParallelAction(
                        hardware.abrirAction(),
                        hardware.ExtendAction(),
                        hardware.inclinadoAction(),
                        hardware.shuparAction(),
                        hardware.elevToPosAction(-45)
                ))
                .splineToLinearHeading(new Pose2d(-48, -55, Math.toRadians(90)), Math.toRadians(180))//posicion agarrar
                .strafeToConstantHeading(new Vector2d(-48, -40), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))

                // DEJAR
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(0.7, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosOnceAction(280),
                        hardware.elevToPosAction(-1650)
                ))
                .afterTime(2, hardware.abrirAction())
                .afterTime(3, hardware.brazoToPosOnceAction(0))
                .strafeToLinearHeading(new Vector2d(-51, -49), Math.toRadians(45))

                .waitSeconds(1.5)
                .afterTime(2, hardware.elevToPosAction(-50))
                .afterTime(1, new ParallelAction(
                        hardware.ExtendAction(),
                        hardware.inclinadoAction(),
                        hardware.shuparAction()
                ))
                .splineToLinearHeading(new Pose2d(-58, -56, Math.toRadians(90)), Math.toRadians(180))//posicion agarrar
                .strafeToConstantHeading(new Vector2d(-58, -40), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 10))

                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(0.7, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosOnceAction(280),
                        hardware.elevToPosAction(-1650)
                ))
                .afterTime(2, hardware.abrirAction())
                .afterTime(4, hardware.brazoToPosOnceAction(0))
                .strafeToLinearHeading(new Vector2d(-51, -49), Math.toRadians(45))
                .waitSeconds(1.5)
                .afterTime(2, hardware.elevToPosAction(-45))
                .afterTime(1, new ParallelAction(
                        hardware.ExtendAction(),
                        hardware.inclinadoAction(),
                        hardware.shuparAction()
                ))


                //agarrar 3
                .splineToLinearHeading(new Pose2d(-45, -52, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(20))
                .strafeToConstantHeading(new Vector2d(-55, -37), null, new ProfileAccelConstraint(-10, 10))

                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(0.7, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosOnceAction(280),
                        hardware.elevToPosAction(-1650),
                        hardware.mantenerAction()
                ))
                .afterTime(2, hardware.abrirAction())
                .strafeToLinearHeading(new Vector2d(-51, -49), Math.toRadians(45))
                .waitSeconds(1.5)
                .afterTime(0.5,new ParallelAction(
                        hardware.elevToPosAction(-45),
                        hardware.brazoToPosOnceAction(0)
                ))
                .splineToLinearHeading(new Pose2d(-22, 13, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, new ParallelAction(
                        hardware.inclinadoAction(),
                        hardware.shuparAction(),
                        hardware.ExtendAction()
                ))
                .waitSeconds(1)
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(1.5, hardware.cerrarAction())
                .afterTime(1.8, new ParallelAction(
                        hardware.elevToPosAction(-1650)
                ))
                .afterTime(2.3,hardware.brazoToPosOnceAction(255))
                .afterTime(3.2, hardware.abrirAction())
               // .strafeToConstantHeading(new Vector2d(-35,-5))
                .strafeToLinearHeading(new Vector2d(-53,-47),Math.toRadians(45))
                .afterTime(1,hardware.elevToPosAction(0))
                .splineToLinearHeading(new Pose2d(-22, -5, Math.toRadians(180)), Math.toRadians(0))
                .build();

        waitForStart();


        hardware.SARelev();
        Actions.runBlocking(new ParallelAction(
                hardware.brazoUpdateAction(),
                hardware.elevUpdateAction(), // update elevador constantemente
                auto,

                telemetryPacket -> {
                    telemetry.addData("brazo", hardware.brazoTargetPos);
                    telemetry.update();
                    return true;
                }
        ));

    }
}

