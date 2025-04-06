package TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hardware {

    public DcMotor elev1;
    public DcMotor elev2;

    public Servo carpus1;
    public Servo carpus2;
    public Servo garra;
    public Servo Ext1;
    public Servo Ext2;
    public Servo Falangs;

    public CRServo G1;
    public CRServo G2;
    public CRServo Brazo;

    public AnalogInput VirtualPos;

    public static PIDController brazoController = new PIDController(0.003, 0, 0);
    public static PIDController brazoControllerDown = new PIDController(0.002, 0, 0);
    public static PIDController elevController = new PIDController(0.016, 0, 0);
    public static PIDController elevControllerDown = new PIDController(0.013, 0, 0);

    public static double armKCos = 0.1;
    public static double FFBrazo = 0.08;

    public double brazoTargetPos = 0;
    public double elevTargetPos = 0;

    public double BrazoP = 0;
    public double lastBrazoP;
    public double brazoPRelative;

    public void init(HardwareMap hardwareMap) {
        elev1 = hardwareMap.dcMotor.get("elev1");
        elev2 = hardwareMap.dcMotor.get("elev2");

        carpus1 = hardwareMap.get(Servo.class, "carpus1");
        carpus2 = hardwareMap.get(Servo.class, "carpus2");
        garra = hardwareMap.get(Servo.class, "garra");
        Ext1 = hardwareMap.get(Servo.class, "Ext1");
        Ext2 = hardwareMap.get(Servo.class, "Ext2");
        Falangs = hardwareMap.get(Servo.class, "falange");

        G1 = hardwareMap.get(CRServo.class, "G1");
        G2 = hardwareMap.get(CRServo.class, "G2");
        Brazo = hardwareMap.get(CRServo.class, "brazo");

        VirtualPos = hardwareMap.get(AnalogInput.class, "Brazoencoder");

        elev1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updateArmPosition() {
        BrazoP = VirtualPos.getVoltage() / 3.3 * 360;
        double deltaPos = BrazoP - lastBrazoP;

        if (Math.abs(deltaPos) < 0.05) deltaPos = 0;
        if (deltaPos > 180) deltaPos -= 360;
        else if (deltaPos < -180) deltaPos += 360;

        brazoPRelative += deltaPos;
        lastBrazoP = BrazoP;
    }

    public void updateArm() {
        updateArmPosition();
        double gravityComp = armKCos * Math.cos(Math.toRadians(brazoPRelative));
        brazoController.setSetPoint(brazoTargetPos);
        double output = brazoController.calculate(brazoPRelative);
        Brazo.setPower(-output + gravityComp);
    }

    public void updateElevator() {
        double pos = elev2.getCurrentPosition();

        if (elevTargetPos > pos) {
            elevControllerDown.setSetPoint(Range.clip(elevTargetPos, -1650, 0));
            double power = elevControllerDown.calculate(pos);
            elev1.setPower(-power);
            elev2.setPower(power);
        } else if (elevTargetPos < pos) {
            elevController.setSetPoint(Range.clip(elevTargetPos, -1650, 0));
            double power = elevController.calculate(pos);
            elev1.setPower(-power);
            elev2.setPower(power);
        } else {
            elev1.setPower(0);
            elev2.setPower(0);
        }
    }

    public Command brazoToPosCommand(int ticks) {
        return new InstantCommand(() -> brazoTargetPos = ticks);
    }

    public Command elevToPosCommand(int ticks) {
        return new InstantCommand(() -> elevTargetPos = ticks);
    }

    public Command setServoPosition(Servo servo, double pos) {
        return new InstantCommand(() -> servo.setPosition(pos));
    }

    public Command setCRServoPower(CRServo servo, double power) {
        return new InstantCommand(() -> servo.setPower(power));
    }

    public Command pickSCommand() {
        return setServoPosition(Falangs, 1);
    }

    public Command dropSCommand() {
        return setServoPosition(Falangs, 0.7);
    }

    public Command extendCommand() {
        return new ParallelCommandGroup(
                setServoPosition(Ext1, 0.7),
                setServoPosition(Ext2, 0.7)
        );
    }

    public Command retractCommand() {
        return new ParallelCommandGroup(
                setServoPosition(Ext1, 0.4),
                setServoPosition(Ext2, 1.0)
        );
    }

    public Command shuparCommand() {
        return new ParallelCommandGroup(
                setCRServoPower(G1, 1),
                setCRServoPower(G2, -1)
        );
    }

    public Command eskupirCommand() {
        return new ParallelCommandGroup(
                setCRServoPower(G1, -1),
                setCRServoPower(G2, 1)
        );
    }

    public Command mantenerCommand() {
        return new ParallelCommandGroup(
                setCRServoPower(G1, 0),
                setCRServoPower(G2, 0)
        );
    }

    public Command posicionInicialCommand() {
        return new ParallelCommandGroup(
                setServoPosition(carpus1, 0.25),
                setServoPosition(carpus2, 0.75)
        );
    }

    public Command inclinadoCommand() {
        return new ParallelCommandGroup(
                setServoPosition(carpus1, 0.51),
                setServoPosition(carpus2, 0.49)
        );
    }

    public Command derechoCommand() {
        return new ParallelCommandGroup(
                setServoPosition(carpus1, 0.4),
                setServoPosition(carpus2, 0.6)
        );
    }

    public Command abrirCommand() {
        return setServoPosition(garra, 0.1);
    }

    public Command cerrarCommand() {
        return setServoPosition(garra, 0.42);
    }
}
