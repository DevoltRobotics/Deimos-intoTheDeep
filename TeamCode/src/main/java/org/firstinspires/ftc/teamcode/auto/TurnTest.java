package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "Turn Test", group = "####")
public class TurnTest extends LinearOpMode {

    PinpointDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        mecanumDrive.pose = new Pose2d(0, 0, Math.toRadians(0));

        Action auto = mecanumDrive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(270))
                        .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(auto);
        }
    }
}
