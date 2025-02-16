package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@TeleOp (name = "lima verde")
public class limelighttest extends LinearOpMode {
    MecanumDrive mecanumDrive;
    Hardware hardware = new Hardware();
    double target;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        hardware.init(hardwareMap);
        telemetry.setMsTransmissionInterval(11);


        hardware.limelight.pipelineSwitch(3);

        /*
         * Starts polling for data.
         */
        hardware.limelight.setPollRateHz(150);
        hardware.limelight.start();
        waitForStart();

        LLResult target = hardware.limelight.getLatestResult();

        if(target != null && target.isValid()) {
            telemetry.addData("tx", target.getTx());
            telemetry.update();
        }
    }
}
