package TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "teleop", group = "##")
public class Teleop extends OpMode {

    MecanumDrive mecanumDrive;
    Hardware hardware = new Hardware();

    enum Virtual { manual, dejar, agarrar, Specimen }
    enum Lift { manual, dejar, agarrar }

    Lift liftstate = Lift.manual;
    Lift prevLiftState = liftstate;
    Virtual virtualstate = Virtual.manual;

    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {
        hardware.updateArmPosition();

        switch (liftstate) {
            case manual:
                if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
                    hardware.elevTargetPos += gamepad2.left_stick_y * 45;
                }
                break;
            case dejar:
                hardware.elevToPosCommand(-1650).execute();
                if (prevLiftState != liftstate) virtualstate = Virtual.dejar;
                break;
            case agarrar:
                if (hardware.BrazoP < 150) {
                    hardware.elevToPosCommand(-80).execute();
                }
                if (prevLiftState != liftstate) virtualstate = Virtual.agarrar;
                break;
        }

        prevLiftState = liftstate;

        switch (virtualstate) {
            case manual:
                if (gamepad2.left_trigger > 0.1) {
                    hardware.brazoTargetPos += gamepad2.left_trigger * 5;
                } else if (gamepad2.right_trigger > 0.1) {
                    hardware.brazoTargetPos -= gamepad2.right_trigger * 5;
                }
                break;
            case dejar:
                hardware.brazoToPosCommand(280).execute();
                break;
            case agarrar:
                hardware.brazoToPosCommand(0).execute();
                break;
            case Specimen:
                hardware.brazoToPosCommand(-460).execute();
                break;
        }

        if (gamepad2.right_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
            virtualstate = Virtual.manual;
        }

        if (gamepad2.right_stick_button) {
            hardware.pickSCommand().execute();
        } else if (gamepad2.left_stick_button) {
            hardware.dropSCommand().execute();
        }

        if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
            liftstate = Lift.manual;
        } else {
            if (gamepad2.dpad_up) {
                liftstate = Lift.dejar;
            } else if (gamepad2.dpad_down) {
                liftstate = Lift.agarrar;
            }
        }

        hardware.updateElevator();
        hardware.updateArm();

        double trigger = Math.max(gamepad1.left_trigger, gamepad1.right_trigger);
        double turbo = 1 - (trigger * 0.65);

        double driveY = -gamepad1.left_stick_y * turbo;
        double driveX = -gamepad1.left_stick_x * turbo;
        double turn = -gamepad1.right_stick_x;

        mecanumDrive.driveRobotCentric(driveX, driveY, turn);

        telemetry.addData("elevador", hardware.elevTargetPos);
        telemetry.addData("extendo1", hardware.Ext1.getPosition());
        telemetry.addData("extendo2", hardware.Ext2.getPosition());
        telemetry.addData("brazotarget", hardware.brazoTargetPos);
        telemetry.addData("brazo", hardware.BrazoP);
        telemetry.addData("brazo relative", hardware.brazoPRelative);

        if (gamepad2.a) {
            hardware.shuparCommand().execute();
        } else if (gamepad2.b) {
            hardware.eskupirCommand().execute();
        } else {
            hardware.mantenerCommand().execute();
        }

        if (gamepad2.dpad_right) {
            hardware.extendCommand().execute();
        } else if (gamepad2.dpad_left) {
            hardware.retractCommand().execute();
        }

        if (gamepad2.y) {
            hardware.posicionInicialCommand().execute();
        } else if (gamepad2.x) {
            hardware.inclinadoCommand().execute();
        } else if (gamepad1.right_bumper) {
            hardware.derechoCommand().execute();
        }

        if (gamepad2.left_bumper) {
            hardware.abrirCommand().execute();
        }

        if (gamepad2.right_bumper) {
            hardware.cerrarCommand().execute();
        }
    }
}
