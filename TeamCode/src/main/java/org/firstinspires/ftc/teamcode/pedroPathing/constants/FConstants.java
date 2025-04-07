package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "lF";
        FollowerConstants.leftRearMotorName = "lB";
        FollowerConstants.rightFrontMotorName = "rF";
        FollowerConstants.rightRearMotorName = "rB";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13;

        FollowerConstants.xMovement = 67.63599440042606;
        FollowerConstants.yMovement = 48.68993412311046;

        FollowerConstants.forwardZeroPowerAcceleration = -39.55808795178483;
        FollowerConstants.lateralZeroPowerAcceleration = -83.12374255140004;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.015,0.02);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.15,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2.1,0,0.13,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0,0,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0,0,0,0,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 300;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}