package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.actions.CRservoAction;
import org.firstinspires.ftc.teamcode.actions.ServoAction;


@Config
public class Hardware {

    MecanumDrive mecanumDrive;


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

    AnalogInput VirtualPos;


    public Limelight3A limelight;


    public static PIDFController.PIDCoefficients brazoCoeffs = new PIDFController.PIDCoefficients(
            0.006, 0, 0
    );

    public PIDFController brazoController = new PIDFController(brazoCoeffs);

    public static PIDFController.PIDCoefficients brazoCoeffsDown = new PIDFController.PIDCoefficients(
            0.007, 0, 0
    );
    public static double FFBrazo = 0.08;

    public PIDFController brazoControllerDown = new PIDFController(brazoCoeffsDown);

    public double brazoTargetPos = 0;

    public static PIDFController.PIDCoefficients rielesCoeffs = new PIDFController.PIDCoefficients(
            0.016, 0, 0
    );

    public PIDFController elevController = new PIDFController(rielesCoeffs);

    public static PIDFController.PIDCoefficients rielesCoeffsDown = new PIDFController.PIDCoefficients(
            0.013, 0, 0
    );

    public PIDFController elevControllerDown = new PIDFController(rielesCoeffsDown);

    public double elevTargetPos = 0;

    public void init(HardwareMap hardwareMap){

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));


        elev1 = hardwareMap.dcMotor.get("elev1");
        elev2 = hardwareMap.dcMotor.get("elev2");

        carpus1 = hardwareMap.get(Servo.class,"carpus1");
        carpus2 = hardwareMap.get(Servo.class,"carpus2");
        garra = hardwareMap.get(Servo.class,"garra");
        Ext1 = hardwareMap.get(Servo.class,"Ext1");
        Ext2 = hardwareMap.get(Servo.class,"Ext2");
        Falangs = hardwareMap.get(Servo.class,"falange");


        G1 = hardwareMap.get(CRServo.class,"G1");
        G2 = hardwareMap.get(CRServo.class,"G2");
        Brazo = hardwareMap.get(CRServo.class,"brazo");

        VirtualPos = hardwareMap.get(AnalogInput.class, "Brazoencoder");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");




        elev1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev1.setDirection(DcMotorSimple.Direction.REVERSE);





        /*
        resto de configuraciones
         */


    }

    double BrazoP = VirtualPos.getVoltage() / 3.3 * 360;



    public void SARelev(){
        elev1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elev2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void shupar(){
        G1.setPower(1);
        G2.setPower(-1);
    }

    public void pickS(){
        Falangs.setPosition(1);
    }

    public ServoAction pickS_Action() {
        return new ServoAction(Falangs, 1);
    }

    public void dropS(){
        Falangs.setPosition(0.7);
    }

    public ServoAction dropS_Action() {
        return new ServoAction(Falangs, 0.7);
    }
    public void Extend (){
        Ext1.setPosition(0.7);
        Ext2.setPosition(0.7);
    }

    public Action ExtendAction(){
        return new ParallelAction(
             new ServoAction( Ext1,0.7),
             new ServoAction( Ext2,0.7)
        );
    }

    public void Rectract(){
        Ext1.setPosition(0.4);
        Ext2.setPosition(1);
    }

    public Action RetractAction(){
        return new ParallelAction(
                new ServoAction( Ext1,0.4),
                new ServoAction( Ext2,1)
        );
    }

    public Action shuparAction (){
        return new ParallelAction(
                new CRservoAction(G1,1),
                new CRservoAction(G2,-1)
        );
    }

    public Action masticarAction (){
        return new ParallelAction(
                new CRservoAction(G1,0),
                new CRservoAction(G2,0)
        );
    }

    public void eskupir (){
        G1.setPower(-1);
        G2.setPower(1);
    }

    public Action eskupirAction(){
        return new ParallelAction(
                new CRservoAction(G1,-1),
                new CRservoAction(G2,1)
        );
    }

    public void mantener(){
        G1.setPower(0);
        G2.setPower(0);
    }

    public Action mantenerAction (){
        return new ParallelAction(
                new CRservoAction(G1,0),
                new CRservoAction(G2,0)
        );
    }






    public void posicion_inicial() {
            carpus1.setPosition(0.27);
            carpus2.setPosition(0.73);
    }

    public Action posicion_inicialAction(){
        return new ParallelAction(
                new ServoAction(carpus1,0.27),
                new ServoAction(carpus2,0.73)
        );
    }

    public void specimen(){
        carpus1.setPosition(0.3);
        carpus2.setPosition(0.7);
    }

    public void inclinado() {
        carpus1.setPosition(0.52);
        carpus2.setPosition(0.48);
    }

    public void derecho(){
        carpus1.setPosition(0.4);
        carpus2.setPosition(0.6);
    }

    public Action inclinadoAction(){
        return new ParallelAction(
          new ServoAction(carpus1,0.52),
          new ServoAction(carpus2,0.48)
        );
    }

    public Action brazoToPosAction(int ticks) {
        return new BrazoToPosAction(ticks);
    }

    public Action brazoToPosSmoothAction(int ticks, double t, double p1) {
        return new BrazoToPosSmoothAction(ticks, t, p1);
    }

    public Action brazoUpdateAction() {
        return new BrazoUpdateAction();
    }


    class BrazoToPosAction implements Action {
        int ticks;

        public BrazoToPosAction(int ticks) {
            this.ticks = ticks;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            brazoTargetPos = ticks; // set target position and end action
            return false;
        }
    }

    class BrazoToPosSmoothAction implements Action {

        int ticks;
        double timeSeconds;
        double p1;

        double currentTicks;

        ElapsedTime timer = null;

        public BrazoToPosSmoothAction(int ticks, double timeSeconds, double p1) {
            this.ticks = ticks;
            this.timeSeconds = timeSeconds;
            this.p1 = p1;
        }

        private double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }
        public double bezier(double p0, double p1, double p2, double t) {
            double q0 = lerp(p0, p1, t);
            double q1 = lerp(p1, p2, t);

            return lerp(q0, q1, t);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                currentTicks = BrazoP;
            }

            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            brazoTargetPos = (int) bezier(currentTicks, p1, ticks, t);

            return timer.seconds() <= timeSeconds;
        }
    }

    class BrazoUpdateAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (brazoTargetPos> BrazoP){
                brazoControllerDown.targetPosition = Range.clip(brazoTargetPos, -360, 360);

                double brazoD = brazoControllerDown.update(BrazoP);
                Brazo.setPower(brazoD);
            }
            if (brazoTargetPos < BrazoP) {
                brazoController.targetPosition = Range.clip(brazoTargetPos, -3000, 0);

                double brazo = brazoController.update(BrazoP);
                Brazo.setPower(brazo);
            }
            return true; // ejecutar por siempre y para siempre
        }
    }

    public void abrir(){
        garra.setPosition(0.1);
    }

    public void cerrar(){
        garra.setPosition(0.42);
    }

    public ServoAction abrirAction() {
        return new ServoAction(garra, 0.1);
    }

    public ServoAction cerrarAction() {
        return new ServoAction(garra, 0.42);
    }


    public ElevToPosAction elevToPosAction(int ticks) {
        return new ElevToPosAction(ticks);
    }

    public ElevUpdateAction elevUpdateAction() {
        return new ElevUpdateAction();
    }






    class ElevToPosAction implements Action {
        int ticks;

        public ElevToPosAction(int ticks) {
            this.ticks = ticks;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elevTargetPos = ticks; // set target position and end action
            return false;
        }
    }

    class ElevUpdateAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (elevTargetPos> elev2.getCurrentPosition()){

                elevControllerDown.targetPosition = Range.clip(elevTargetPos, -1650, 0);

                double elevD = elevControllerDown.update(elev2.getCurrentPosition());
                elev1.setPower(-elevD);
                elev2.setPower(elevD);
            }


            if (elevTargetPos< elev2.getCurrentPosition()) {

                elevController.targetPosition = Range.clip(elevTargetPos, -1650, 0);

                double elev = elevController.update(elev2.getCurrentPosition());
                elev1.setPower(-elev);
                elev2.setPower(elev);

            }
            return true; // ejecutar por siempre y para siempre
        }
    }


}
