package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.PIDFController;

@Config
public class Constants {
    public static PIDFController.PIDCoefficients chassisCoefficients = new PIDFController.PIDCoefficients(0.12, 0, 0.02);

    public static double ScaleFactor = 80;

    public static int xPixels = 320;
    public static int xFov = 20;
    public static double xDegreesPerPixel = (double) (xPixels) / (xFov);

    public static int yPixels = 240;
    public static int yFov = 10;

    public static double hCamera = 5.1;
    public static double intakeAngle = 30.0; // en grados;

    public static double intakeAngleRode = 25.0; // en grados;

    public static int correction = 100; // en grados;

    public static double getTargetExtensionFromY(double tY, double currentRodeTicks){
        double yDegreesPerPixel = (double) (yFov) / (yPixels);
        double offsetFromCenter  = (yPixels) - tY;
        double viewAngle  = offsetFromCenter  * yDegreesPerPixel;

        double totalAngleDeg = viewAngle + ((viewAngle / yFov) * intakeAngleRode);
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        double extensionInches = hCamera * Math.tan(totalAngleRad);

        double rodeTarget = (currentRodeTicks - (extensionInches * rodeInToTicks) - correction);

        int ticks = Range.clip((int)(rodeTarget), preSubmRodePos - 600, preSubmRodePos);

        return ticks;
    }

    public static double getTargetAngleY(double tY){
        double yDegreesPerPixel = (double) (yFov) / (yPixels);
        double offsetFromCenter  = (yPixels) - tY;
        double viewAngle  = offsetFromCenter  * yDegreesPerPixel;

        return viewAngle;
    }

    public static final int ratioArm = 20;

    public static double openAbramPos = 0.05;
    public static double midOpenAbramPos = 0.25;
    public static double contractAbramPos = 0.5;

    public static double closeClawPos = 1;
    public static double openClawPos = 0;

    public static double preSubWristPos = 1;
    public static double initWristPos = 0.8;
    public static double SubWristPos = 1;
    public static double pickSubWristPos = 0.83;
    public static double downWristPos = 0.6;
    public static double pickSpecimenWristPos = 0.64;
    public static double specimenWristPos = 0.47;
    public static double contractWristPos = 0.27;
    public static double mediumWristPos = 0.3;
    public static double firstSpecimenWristPos = 0.47;
    public static double previousSpecimenWristPos = 0.1;
    public static double basketWristPos = 0;

    public static int submArmPos = -300;
    public static int initArmPos = -950;
    public static int firstSpecimenArmPos = -850;
    public static int specimenArmPos = -2050;
    public static int postSpecimenArmPos = -1200;
    public static int basketArmPos = -1900;

    public static int highRodePos = -1200; //
    public static int preSubmRodePos = -500; //
    public static int postSubmRodePos = -1050; //
    public static int firstSpecimenRodePos = -1100;
    public static int specimenRodePos = -700;
    public static int preSpecimenRodePos = -250;
    public static int downRodePos = -750; //
    public static int climbingRodePos2 = -390;
    public static int climbingRodePos1 = -200;

    public static double servosHangingPos = 0.2;
    public static double servosClimbingPos = 0.1;
    public static double servosInitPos = -0.05;
    public static double servosTestPost = 0;

    public static double launchArmsPos = 0.5;
    public static double supportArmsPos = -0.4;

    public static double rodeTick = 384.5;
    public static double rodeCircIn = 3.71;
    public static double rodeTicksToIn = rodeCircIn / rodeTick;
    public static double rodeInToTicks = rodeTick / rodeCircIn;

}