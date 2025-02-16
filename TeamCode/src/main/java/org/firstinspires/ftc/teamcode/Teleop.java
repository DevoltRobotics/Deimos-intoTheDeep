package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "teleop", group = "##")
public class Teleop extends OpMode {

    MecanumDrive mecanumDrive;
    Hardware hardware = new Hardware();

    Action brazoUpdateAction = hardware.brazoUpdateAction();
    Action elevUpdateAction = hardware.elevUpdateAction();
    Action brazodownsmoothaction ;
    Action brazoupsmoothaction;


    enum virtual {
        manual,
        dejar,
        agarrar,
        Specimen
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
        telemetry.setMsTransmissionInterval(11);

        hardware.limelight.pipelineSwitch(3);

        /*
         * Starts polling for data.
         */
        hardware.limelight.start();
    }

    @Override
    public void loop() {


            LLResult result = hardware.limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    double targetOffsetAngle_Vertical = result.getTy();

                    // how many degrees back is your limelight rotated from perfectly vertical?
                    double limelightMountAngleDegrees = 32.0;

                    // distance from the center of the Limelight lens to the floor
                    double limelightLensHeightInches = 4.0;

                    // distance from the target to the floor
                    double goalHeightInches = 0;

                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                    //calculate distance
                    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                    telemetry.addData("distance", distanceFromLimelightToGoalInches);
                }
            } else {
        telemetry.addData("Limelight", "No Targets");
        }



                    switch (liftstate) {
            case manual:
                if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
                    hardware.elevTargetPos += (gamepad2.left_stick_y) * 45;
                }
                break;
            case dejar:


                    hardware.elevToPosAction(-1650).run(null);

                if(prevLiftState != liftstate) {
                    virtualstate = virtual.dejar;
                }
                break;
            case agarrar:
        if (hardware.virtual.getCurrentPosition() > -150) {
            hardware.elevToPosAction(-105).run(null);
        }
                if(prevLiftState != liftstate) {
                    virtualstate = virtual.agarrar;
                }
        }

        prevLiftState = liftstate;

        switch (virtualstate) {
            case manual:
                if (gamepad2.left_trigger > 0.1) {
                    hardware.brazoTargetPos += (gamepad2.left_trigger) * 15;
                } else if (gamepad2.right_trigger > 0.1) {
                    hardware.brazoTargetPos -= (gamepad2.right_trigger) * 15;
                }


                break;
            case dejar:
                if (hardware.elev2.getCurrentPosition() < -200) {
                    brazoupsmoothaction.run(null);
                }
                break;
            case agarrar:
                brazodownsmoothaction.run(null);
                break;
            case Specimen:
                hardware.brazoToPosAction(-460).run(null);
        }

        if (gamepad2.right_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
            virtualstate = virtual.manual;
        }

        if (gamepad2.right_stick_button){
            hardware.pickS();
        } else if (gamepad2.left_stick_button) {
            hardware.dropS();
        }

        if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
            liftstate = lift.manual;
        } else {
            if (gamepad2.dpad_up) {
                liftstate = lift.dejar;
                brazoupsmoothaction = hardware.brazoToPosSmoothAction(-375, 1, -250);

            } else if (gamepad2.dpad_down) {
                liftstate = lift.agarrar;
                brazodownsmoothaction = hardware.brazoToPosSmoothAction(0, 0.5, -50);
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
        telemetry.addData("brazotarget",hardware.brazoTargetPos);
        telemetry.addData("brazo",hardware.virtual.getCurrentPosition());

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
