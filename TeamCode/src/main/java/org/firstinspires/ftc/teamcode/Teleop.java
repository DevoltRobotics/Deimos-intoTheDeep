package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "teleop")
public class Teleop extends OpMode {

    MecanumDrive mecanumDrive;
    Hardware hardware = new Hardware();

    Action brazoUpdateAction = hardware.brazoUpdateAction();
    Action elevUpdateAction = hardware.elevUpdateAction();

    enum virtual {
        manual,
        dejar,
        agarrar
    }

    enum lift {
        manual,
        dejar,
        agarrar

    }

    lift liftstate = lift.manual;
    lift prevLiftState = liftstate;

    virtual virtualstate = virtual.manual;

    @Override
    public void init() {
        mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {

        switch (liftstate) {
            case manual:
                if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
                    hardware.elevTargetPos += (gamepad2.left_stick_y) * 45;
                }
                break;
            case dejar:
                hardware.elevToPosAction(-2600).run(null);

                if(prevLiftState != liftstate) {
                    virtualstate = virtual.dejar;
                }
                break;
            case agarrar:
                hardware.elevToPosAction(-13).run(null);

                if(prevLiftState != liftstate) {
                    virtualstate = virtual.agarrar;
                }
        }

        prevLiftState = liftstate;

        switch (virtualstate) {
            case manual:
                if (gamepad2.left_trigger > 0.1) {
                    hardware.brazoTargetPos += (gamepad2.left_trigger) * 12;
                } else if (gamepad2.right_trigger > 0.1) {
                    hardware.brazoTargetPos -= (gamepad2.right_trigger) * 12;
                }


                break;
            case dejar:
                if (hardware.elev2.getCurrentPosition() < -1800) {
                    hardware.brazoToPosAction(-365).run(null);
                }
                break;
            case agarrar:
                hardware.brazoToPosAction(0).run(null);
                break;

        }

        if (gamepad2.right_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
            virtualstate = virtual.manual;
        }

        if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
            liftstate = lift.manual;
        } else {
            if (gamepad2.dpad_up) {
                liftstate = lift.dejar;
            } else if (gamepad2.dpad_down) {
                liftstate = lift.agarrar;
            }
        }

        if (gamepad1.y){
            hardware.SARbrazo();
        }

        elevUpdateAction.run(null);
        brazoUpdateAction.run(null);

        double trigger = gamepad1.left_trigger;
        if (gamepad1.right_trigger > 0.35) {
            trigger = gamepad1.right_trigger;
        }

        double turbo = 1 - (trigger * 0.65);

        mecanumDrive.updatePoseEstimate();

        Vector2d drive = new Vector2d(-gamepad1.left_stick_y * turbo, -gamepad1.left_stick_x * turbo);
        drive = mecanumDrive.pose.heading.inverse().times(drive);

        mecanumDrive.setDrivePowers(new PoseVelocity2d(drive, -gamepad1.right_stick_x));

        if (gamepad1.dpad_left) {
            mecanumDrive.pose = (new Pose2d(0, 0, 0));
        }

        telemetry.addData("imu", Math.toDegrees(mecanumDrive.pose.heading.toDouble()));
        telemetry.addData("elevador", hardware.elevTargetPos);
        telemetry.addData("extendo1", hardware.Ext1.getPosition());
        telemetry.addData("extendo2", hardware.Ext2.getPosition());


        if (gamepad2.a) {
            hardware.shupar();
        } else if (gamepad2.b) {
            hardware.eskupir();
        } else {
            hardware.mantener();
        }

        if (gamepad2.dpad_right) {
            hardware.Extend();
        } else if (gamepad2.dpad_left) {
            hardware.Rectract();
        }


        if (gamepad2.y) {
            hardware.posicion_inicial();
        } else if (gamepad2.x) {
            hardware.inclinado();
        } else if (gamepad1.right_bumper) {
            hardware.derecho();
        }


        if (gamepad2.left_bumper) {
            hardware.abrir();
        }

        if (gamepad2.right_bumper) {
            hardware.cerrar();
        }

    }
}
