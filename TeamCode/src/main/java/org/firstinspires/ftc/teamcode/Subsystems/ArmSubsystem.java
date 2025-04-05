package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.PIDFController;

public class ArmSubsystem extends SubsystemBase {
    public CRServo arm;
    public AnalogInput ArmPos;

    public static PIDFController.PIDCoefficients brazoCoeffs = new PIDFController.PIDCoefficients(
            0.0032, 0, 0
    );

    public static double armKCos = 0.05;
    public PIDFController brazoController = new PIDFController(brazoCoeffs);

    public double BrazoP = 0;
    public double lastBrazoP;

    public double brazoPRelative;

    public int ticks;

    public double brazoTargetPos;

    public int scorePos = 280;

    public ArmSubsystem(HardwareMap hMap) {
        arm = hMap.get(CRServo.class, "brazo");
        ArmPos = hMap.get(AnalogInput.class, "brazoPos");
    }


    @Override
    public void periodic() {

        BrazoP = ArmPos.getVoltage() / 3.3 * 360;

        double deltaPos = BrazoP - lastBrazoP;

        if(Math.abs(deltaPos) < 0.05) {
            deltaPos = 0;
        }

        if (deltaPos > 180) {
            deltaPos -= 360;
        } else if (deltaPos < -180) {
            deltaPos += 360;
        }

        brazoPRelative += deltaPos;

        lastBrazoP = BrazoP;

        brazoTargetPos = ticks;

        double gravityComp = armKCos * Math.cos(Math.toRadians(brazoPRelative -20));
        brazoController.targetPosition = brazoTargetPos;
        double brazo = brazoController.update(brazoPRelative);
        arm.setPower(-brazo + gravityComp);

    }
}
