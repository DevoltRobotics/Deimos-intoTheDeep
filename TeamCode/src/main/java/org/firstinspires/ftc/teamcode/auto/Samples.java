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

@Autonomous(name = "autonuevo",group = "###")
public class Samples extends LinearOpMode {

    PinpointDrive mecanumDrive;
    Hardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, -61, Math.toRadians(0));

        mecanumDrive = new PinpointDrive(hardwareMap, startPose);

        hardware = new Hardware();
        hardware.init(hardwareMap);

        hardware.SARelev();


        Action auto = mecanumDrive.actionBuilder(startPose)

                .afterTime(0, new ParallelAction(
                        hardware.cerrarAction(),
                        hardware.elevToPosOnceAction(-1650),
                        hardware.posicion_inicialAction()
                ))
                .afterTime(0, hardware.brazoToPosOnceAction(250))
                .afterTime(1.5, new ParallelAction(
                        hardware.abrirAction(),
                        hardware.brazoToPosAction(0)
                ))
                .strafeToLinearHeading(new Vector2d(-52,-50), Math.toRadians(45))//posicion para poner
                .waitSeconds(.5)
                .afterTime(1, hardware.brazoToPosOnceAction(0))
                .afterTime(1, new ParallelAction(
                        hardware.abrirAction(),
                        hardware.ExtendAction(),
                        hardware.inclinadoAction(),
                        hardware.shuparAction(),
                        hardware.elevToPosOnceAction(-65)
                ))
                .splineToLinearHeading(new Pose2d(-48,-55, Math.toRadians(90)), Math.toRadians(180))//posicion agarrar
                .strafeToConstantHeading(new Vector2d(-48,-40),new TranslationalVelConstraint(20),new ProfileAccelConstraint(-10,10))

                // DEJAR
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(0.7, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(250),
                        hardware.elevToPosAction(-1650)
                ))
                .afterTime(2, hardware.abrirAction())
                .afterTime(3.5, hardware.brazoToPosAction(0))
                .strafeToLinearHeading(new Vector2d(-51,-49), Math.toRadians(45))

                .waitSeconds(1.5)
                .afterTime(2, hardware.elevToPosAction(-70))
                .afterTime(1,new ParallelAction(
                        hardware.ExtendAction(),
                        hardware.inclinadoAction(),
                        hardware.shuparAction()
                ))
                .splineToLinearHeading(new Pose2d(-58,-56, Math.toRadians(90)), Math.toRadians(180))//posicion agarrar
                .strafeToConstantHeading(new Vector2d(-58,-40),new TranslationalVelConstraint(20),new ProfileAccelConstraint(-10,10))

                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(0.7, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(250),
                        hardware.elevToPosAction(-1650)
                ))
                .afterTime(2, hardware.abrirAction())
                .afterTime(4, hardware.brazoToPosAction(0))
                .strafeToLinearHeading(new Vector2d(-51,-49), Math.toRadians(45))
                .waitSeconds(1.5)
                .afterTime(2, hardware.elevToPosAction(-65))
                .afterTime(1,new ParallelAction(
                        hardware.ExtendAction(),
                        hardware.inclinadoAction(),
                        hardware.shuparAction()
                ))


                //agarrar 3
                .splineToLinearHeading(new Pose2d(-45,-52, Math.toRadians(135)), Math.toRadians(180), new TranslationalVelConstraint(20))
                .strafeToConstantHeading(new Vector2d(-54,-38),null,new ProfileAccelConstraint(-10,10))

                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(), // carpus
                        hardware.RetractAction()
                ))
                .afterTime(0.7, hardware.cerrarAction())
                .afterTime(1, new ParallelAction(
                        hardware.brazoToPosAction(250),
                        hardware.elevToPosAction(-1650),
                        hardware.mantenerAction()
                ))
                .afterTime(2, hardware.abrirAction())
                .strafeToLinearHeading(new Vector2d(-51,-49), Math.toRadians(45))
                .waitSeconds(1.5)
                .afterTime(0.5,hardware.elevToPosAction(0))
                .splineToLinearHeading(new Pose2d(-20,-5, Math.toRadians(180)), Math.toRadians(0))
                .build();

        waitForStart();


        hardware.SARelev();

        Actions.runBlocking(new ParallelAction(
                auto,
                hardware.elevUpdateAction(), // update elevador constantemente
                hardware.brazoUpdateAction(),

                telemetryPacket -> {

                    telemetry.addData("brazo",hardware.brazoTargetPos);
                    telemetry.update();
                    return false;
                }
        ));
    }
}
