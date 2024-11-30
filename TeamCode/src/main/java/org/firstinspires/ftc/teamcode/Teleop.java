package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "teleop")
public class Teleop extends OpMode {

    MecanumDrive mecanumDrive;
    Hardware hardware;
    int targetposition;
    int targetpositionext;


    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        hardware = new Hardware();
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
            targetposition -= (gamepad2.left_stick_y) * 45 ;
        } else {
            if (gamepad2.dpad_up) {
                targetposition = 1400;
            } else if (gamepad2.dpad_down) {
                targetposition = 0;
            }
        }

        if (Math.abs(gamepad2.right_stick_y) >= 0.1) {
            targetpositionext += (gamepad2.right_stick_y) * 45;
        } else {
            if (gamepad2.dpad_right) {
                targetpositionext = 700;
            } else if (gamepad2.dpad_left) {
                targetpositionext = 0;
            }
        }

        hardware.Extend(1,targetpositionext);

        double trigger = gamepad1.left_trigger;

        if (gamepad1.right_trigger > 0.35) {
            trigger = gamepad1.right_trigger;
        }

        double turbo = 1 - (trigger * 0.65);

        mecanumDrive.updatePoseEstimate();

        Vector2d drive = new Vector2d(-gamepad1.left_stick_y * turbo, -gamepad1.left_stick_x * turbo);
        drive = mecanumDrive.pose.heading.inverse().times(drive);

        mecanumDrive.setDrivePowers(new PoseVelocity2d(drive,-gamepad1.right_stick_x));

        if (gamepad1.dpad_left) {
            mecanumDrive.pose = (new Pose2d(0,0,0));
        }

        telemetry.addData("imu", Math.toDegrees(mecanumDrive.pose.heading.toDouble()));

        telemetry.addData("elevador",targetposition);
        telemetry.addData("extendo",targetpositionext);

        if(gamepad2.a){
            hardware.shupar();
        } else if (gamepad2.b) {
            hardware.eskupir();
        }else {
         hardware.mantener();
        }

        if (gamepad2.y){
            hardware.posicion_inicial();
        } else if (gamepad2.x) {
            hardware.inclinado();
        }


        
        if (gamepad2.left_trigger > 0.3){
            hardware.tomar(1);
        } else if (gamepad2.right_trigger > 0.3) {
            hardware.dejar(1);
        }else {
            hardware.tomar(0);
        }


        if (gamepad2.left_bumper){
            hardware.abrir();
        }

        if (gamepad2.right_bumper){
            hardware.cerrar();
        }
        hardware.Elev(1,targetposition);
    }
}
