package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {


    public DcMotor extendo;

    public DcMotor elev;


    public Servo carpus1;
    public Servo carpus2;
    public Servo garra;



    public CRServo G1;
    public CRServo G2;
    public CRServo brazo;

    public void init(HardwareMap hardwareMap){

        extendo = hardwareMap.dcMotor.get("extendo");
        elev = hardwareMap.dcMotor.get("elev");

        carpus1 = hardwareMap.get(Servo.class,"carpus1");
        carpus2 = hardwareMap.get(Servo.class,"carpus2");
        garra = hardwareMap.get(Servo.class,"garra");



        brazo = hardwareMap.get(CRServo.class,"brazo");
        G1 = hardwareMap.get(CRServo.class,"G1");
        G2 = hardwareMap.get(CRServo.class,"G2");


        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void shupar(){
        G1.setPower(1);
        G2.setPower(-1);
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

    public void specimen(){
        carpus1.setPosition(0.3);
        carpus2.setPosition(0.7);
    }

    public void inclinado() {
        carpus1.setPosition(0.7);
        carpus2.setPosition(0.3);
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







}
