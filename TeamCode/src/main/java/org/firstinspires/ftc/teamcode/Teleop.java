package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp (name = "teleop")
public class Teleop extends OpMode {



    ElapsedTime timerColgar = new ElapsedTime();
    boolean voltageIndicatorBoolean;
    boolean noForzarServos = false;
    MecanumDrive mecanumDrive;
    Hardware hardware;
    int targetposition;
    int targetpositionext;
    boolean modo = false;
    boolean manualRight;
    boolean manualLeft;


    @Override
    public void init() {
        mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        hardware = new Hardware();
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1){
            modo = true;
        }

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
        } else {
            if (gamepad2.dpad_right) {
                targetpositionext = 700;
            } else if (gamepad2.dpad_left) {
                targetpositionext = 0;
            }
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
        telemetry.addData("extendo",targetpositionext);
        telemetry.addData("brazo pos", hardware.GH2.getCurrentPosition());
        telemetry.addData("voltaje",hardware.GH2.getCurrent(CurrentUnit.AMPS));

        if(gamepad2.a){
            hardware.shupar();
        } else if (gamepad2.b) {
            hardware.eskupir();
        }else {
         hardware.mantener();
        }

        if (modo) {
            manualRight = Math.abs(gamepad2.right_stick_y) > 0.5;
            manualLeft = Math.abs(gamepad2.left_stick_y) > 0.5;



                hardware.GH1.setPower(gamepad2.right_stick_y);
                




                hardware.GH2.setPower(gamepad2.left_stick_y);





            if (manualRight && manualLeft){
                voltageIndicatorBoolean = true;
                timerColgar.reset();

                noForzarServos = false;

            }

            if ((Math.abs(hardware.GH2.getCurrent(CurrentUnit.AMPS)) > 1 && Math.abs(hardware.GH2.getCurrent(CurrentUnit.AMPS)) > 1  && timerColgar.seconds() > 0.2 )){
                hardware.ServoDown();

                noForzarServos = true;


                telemetry.addLine("motor_forzado");
            }
            hardware.Elev(0,targetposition);
            hardware.extendo.setPower(0);
        }

        if (gamepad1.right_bumper){
            hardware.ServoUp();
        }

        if (gamepad2.y){
            hardware.posicion_inicial();
        } else if (gamepad2.x) {
            hardware.inclinado();
        }


        if (modo == false) {
            hardware.Elev(1,targetposition);
            hardware.extendo.setPower(gamepad2.right_stick_y);
        }

        if (gamepad2.left_trigger > 0.3) {
            hardware.tomar(1);
        } else if (gamepad2.right_trigger > 0.3) {
            hardware.dejar(1);
        } else {
            hardware.tomar(0);
        }

        if (gamepad2.left_bumper){
            hardware.abrir();
        }

        if (gamepad2.right_bumper){
            hardware.cerrar();
        }

    }
}
