package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.CRservoAction;
import org.firstinspires.ftc.teamcode.actions.ServoAction;

@Config
public class Hardware {

    MecanumDrive mecanumDrive;


    public DcMotor elev1;
    public DcMotor elev2;
    public DcMotor virtual;


    public Servo carpus1;
    public Servo carpus2;
    public Servo garra;
    public Servo Ext1;
    public Servo Ext2;

    public CRServo G1;
    public CRServo G2;


    public static PIDFController.PIDCoefficients brazoCoeffs = new PIDFController.PIDCoefficients(
            0.001, 0, 0
    );

    public PIDFController brazoController = new PIDFController(brazoCoeffs);

    public static PIDFController.PIDCoefficients rielesCoeffs = new PIDFController.PIDCoefficients(
            0.01, 0, 0
    );

    public PIDFController rielesController = new PIDFController(rielesCoeffs);
    public double rielesTargetPos = 0;

    public void init(HardwareMap hardwareMap){

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));


        elev1 = hardwareMap.dcMotor.get("elev1");
        elev2 = hardwareMap.dcMotor.get("elev2");
        virtual = hardwareMap.dcMotor.get("virtual");

        carpus1 = hardwareMap.get(Servo.class,"carpus1");
        carpus2 = hardwareMap.get(Servo.class,"carpus2");
        garra = hardwareMap.get(Servo.class,"garra");
        Ext1 = hardwareMap.get(Servo.class,"Ext1");
        Ext2 = hardwareMap.get(Servo.class,"Ext2");


        G1 = hardwareMap.get(CRServo.class,"G1");
        G2 = hardwareMap.get(CRServo.class,"G2");


        /*
        reset encoders
         */

        elev1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elev2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        virtual.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        /*
        resto de configuraciones
         */


    }

    public void shupar(){
        G1.setPower(1);
        G2.setPower(-1);
    }

    public void Extend (){
        Ext1.setPosition(0.7);
        Ext2.setPosition(0.7);
    }

    public void Rectract(){
        Ext1.setPosition(0.4);
        Ext2.setPosition(1);
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

    public void Virtual(double power){
        virtual.setPower(power);
    }

   public void Elev(double power, int ticks){
        elev1.setTargetPosition(ticks);
        elev1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev1.setPower(power);
        elev2.setTargetPosition(-ticks);
        elev2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elev2.setPower(power);
    }

    public void Extend(double power, int ticks){
        extendo.setTargetPosition(ticks);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(power);
    }

    public void ServoDown(){
        izq.setPosition(1);
        der.setPosition(0);
    }
    public void ServoUp(){
        izq.setPosition(0);
        der.setPosition(1);
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
        carpus1.setPosition(0.72);
        carpus2.setPosition(0.28);
    }

    public Action inclinadoAction(){
        return new ParallelAction(
          new ServoAction(carpus1,0.72),
          new ServoAction(carpus2,0.28)
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

    public ElevToPosAction elevToPosAction(int ticks) {
        return new ElevToPosAction(ticks);
    }

    public ElevUpdateAction elevUpdateAction() {
        return new ElevUpdateAction();
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

    class ElevToPosAction implements Action {
        int ticks;

        public ElevToPosAction(int ticks) {
            this.ticks = ticks;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rielesTargetPos = ticks; // set target position and end action
            return false;
        }
    }

    class ElevUpdateAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rielesController.targetPosition = rielesTargetPos;
            elev1.setPower(rielesController.update(elev1.getCurrentPosition()));

            return true; // ejecutar por siempre
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
