package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.CRservoAction;
import org.firstinspires.ftc.teamcode.actions.ServoAction;

@Config
public class Hardware {

    MecanumDrive mecanumDrive;

    public DcMotor extendo;
    public DcMotor elev;
    public DcMotor GH1;
    public DcMotor GH2;

    public Servo carpus1;
    public Servo carpus2;
    public Servo garra;
    public Servo izq;
    public Servo der;

    public CRServo G1;
    public CRServo G2;
    public CRServo brazo;

    public static PIDFController.PIDCoefficients brazoCoeffs = new PIDFController.PIDCoefficients(
            0.001, 0, 0
    );

    public PIDFController brazoController = new PIDFController(brazoCoeffs);

    public void init(HardwareMap hardwareMap){

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        extendo = hardwareMap.dcMotor.get("extendo");
        elev = hardwareMap.dcMotor.get("elev");
        GH1 = hardwareMap.dcMotor.get("GH1");
        GH2 = hardwareMap.dcMotor.get("GH2");

        carpus1 = hardwareMap.get(Servo.class,"carpus1");
        carpus2 = hardwareMap.get(Servo.class,"carpus2");
        garra = hardwareMap.get(Servo.class,"garra");
        izq = hardwareMap.get(Servo.class,"izq");
        der = hardwareMap.get(Servo.class,"der");

        brazo = hardwareMap.get(CRServo.class,"brazo");
        G1 = hardwareMap.get(CRServo.class,"G1");
        G2 = hardwareMap.get(CRServo.class,"G2");

        GH1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GH2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GH2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GH2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void shupar(){
        G1.setPower(1);
        G2.setPower(-1);
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

    public void mantener(){
        G1.setPower(0);
        G2.setPower(0);
    }

   public void Elev(double power, int ticks){
        elev.setTargetPosition(ticks);
        elev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev.setPower(power);
    }

    public void Extend(double power, int ticks){
        extendo.setTargetPosition(ticks);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(power);
    }

    public void posicion_inicial() {
            carpus1.setPosition(0.45);
            carpus2.setPosition(0.55);
    }

    public Action posicion_inicialAction(){
        return new ParallelAction(
                new ServoAction(carpus1,0.45),
                new ServoAction(carpus2,0.55)
        );
    }

    public void specimen(){
        carpus1.setPosition(0.3);
        carpus2.setPosition(0.7);
    }

    public void inclinado() {
        carpus1.setPosition(0.7);
        carpus2.setPosition(0.3);
    }

    public Action inclinadoAction(){
        return new ParallelAction(
          new ServoAction(carpus1,0.7),
          new ServoAction(carpus2,0.3)
        );
    }

    public void colgarse(double power){
        GH1.setPower(power);
        GH2.setPower(power);
    }

    public void tomar(double power){
        brazo.setPower(-power);
    }

    public void dejar(double power){
        brazo.setPower(power);
    }

    public void abrir(){
        garra.setPosition(0);
    }

    public void cerrar(){
        garra.setPosition(0.37);
    }


    public ServoAction abrirAction() {
        return new ServoAction(garra, 0);
    }

    public ServoAction cerrarAction() {
        return new ServoAction(garra, 0.37);
    }

    public ExtendAction extendAction(int ticks, double power) {
        return new ExtendAction(ticks, power);
    }

    public BrazoToPosAction brazoToPosAction(int targetPos) {
        return new BrazoToPosAction(targetPos);
    }

    class ExtendAction implements Action {
        int ticks;
        double power;

        public ExtendAction(int ticks, double power) {
            this.ticks = ticks;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendo.setTargetPosition(ticks);
            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendo.setPower(power);

            return extendo.isBusy();
        }
    }

    class BrazoToPosAction implements Action {
        double targetPos;

        public BrazoToPosAction(double targetPos) {
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            brazoController.targetPosition = targetPos;
            brazo.setPower(-brazoController.update(GH2.getCurrentPosition()));

            return Math.abs(brazoController.lastError) > 50;
        }
    }
}
