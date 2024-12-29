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
        mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        hardware = new Hardware();
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {



        if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
            targetposition -= (gamepad2.left_stick_y) * 45 ;
        } else {
            if (gamepad2.dpad_up) {
                targetposition = 1400*3;
            } else if (gamepad2.dpad_down) {
                targetposition = 0;
            }
        }

        if (Math.abs(gamepad2.right_stick_y) >= 0.1) {
            targetpositionext += (int) ((gamepad2.right_stick_y) * 45);
        }


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
        telemetry.addData("extendo1",hardware.Ext1.getPosition());
        telemetry.addData("extendo2",hardware.Ext2.getPosition());

        if(gamepad2.a){
            hardware.shupar();
        } else if (gamepad2.b) {
            hardware.eskupir();
        }else {
         hardware.mantener();
        }

        if (gamepad2.dpad_right){
            hardware.Extend();
        } else if (gamepad2.dpad_left) {
            hardware.Rectract();
        }




        if (gamepad2.y){
            hardware.posicion_inicial();
        } else if (gamepad2.x) {
            hardware.inclinado();
        }



            hardware.Elev(1,targetposition);



        if (gamepad2.left_trigger > 0.1) {
            hardware.Virtual(-gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > 0.1) {
            hardware.Virtual(gamepad2.right_trigger);
        } else {
            hardware.Virtual(0);
        }

        if (gamepad2.left_bumper){
            hardware.abrir();
        }

        if (gamepad2.right_bumper){
            hardware.cerrar();
        }

    }
}
