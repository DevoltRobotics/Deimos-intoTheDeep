package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.PIDFController;


public class ElevatorSubsystem extends SubsystemBase {

    public DcMotor ElevL, ElevR;

    public static PIDFController.PIDCoefficients rielesCoeffs = new PIDFController.PIDCoefficients(
            0.016, 0, 0
    );

    public PIDFController elevController = new PIDFController(rielesCoeffs);

    public int ticks;

    public static final int ScorePos = 1400;
    public static final int TransferPos = 50;

    public ElevatorSubsystem(final HardwareMap hMap) {
        ElevL = hMap.get(DcMotor.class, "ElevL");
        ElevR = hMap.get(DcMotor.class, "ElevR");

        ElevL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ElevR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SARelev() {
        ElevL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        elevController.targetPosition = Range.clip(ticks, 0, 1500);

        double ElevD = elevController.update(ElevR.getCurrentPosition());
        ElevR.setPower(ElevD);
        ElevL.setPower(-ElevD);
    }
}
