package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Azules", group = "####")
public class azules extends LinearOpMode {

    PinpointDrive mecanumDrive;
    Hardware hardware;


    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        hardware = new Hardware();
        hardware.init(hardwareMap);

        mecanumDrive.pose = new Pose2d(-8, -62, Math.toRadians(270));

        Action auto = mecanumDrive.actionBuilder(new Pose2d(-8, -62, Math.toRadians(270)))
                .afterTime(0, new ParallelAction(
                        hardware.brazoToPosAction(-6700),
                        hardware.cerrarAction()
                ) )
                .waitSeconds(.5)
                .strafeToConstantHeading(new Vector2d(-8,-41))
                .waitSeconds(.5)
                .strafeToConstantHeading(new Vector2d(-8,-38))
                .afterTime(0.5,hardware.brazoToPosAction(-5100))
                .waitSeconds(2)
                .afterTime(0,new ParallelAction(
                        hardware.abrirAction(),
                        hardware.extendAction(700,1),
                        hardware.inclinadoAction(),
                        hardware.shuparAction(),
                        hardware.brazoToPosAction(0)
                ))
                .splineToLinearHeading(new Pose2d(-48,-47, Math.toRadians(90)), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-48,-40))
                .waitSeconds(1)
                .afterTime(0, new ParallelAction(
                        hardware.posicion_inicialAction(),
                        hardware.extendAction(0,1)
                ))
                .strafeToLinearHeading(new Vector2d(-54,-52), Math.toRadians(45))
                .build();

        waitForStart();
        telemetry.addData("imu", Math.toDegrees(mecanumDrive.pose.heading.toDouble()));
        telemetry.update();
        Actions.runBlocking(auto);
    }
}
