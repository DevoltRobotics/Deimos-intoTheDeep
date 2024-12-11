package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = -2000.6959937156323; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 240.34799685781616; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, IMU navx, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "lB")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "lF")));

        // TODO: reverse encoder directions if needed
        par.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    double lastAngle;
    double lastAngularRate;
    ElapsedTime deltaTimer = new ElapsedTime();
    boolean firstUpdate = true;

    public Twist2dDual<Time> update() {
        if(firstUpdate) {
            deltaTimer.reset();
            firstUpdate = false;
        }

        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        double currentAngle = 0;//-navx.getYaw();
        lastAngularRate = ((currentAngle - lastAngle + 180) % 360 - 180) / deltaTimer.seconds();
        lastAngle = currentAngle;
        deltaTimer.reset();

        YawPitchRollAngles angles = new YawPitchRollAngles(
                AngleUnit.RADIANS, Math.toRadians(lastAngle), 0, 0,+
                System.currentTimeMillis()
        );

        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                0,0, (float)Math.toRadians(lastAngularRate),
                System.currentTimeMillis()
        );

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(Math.toRadians(currentAngle));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = lastAngularRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = (int) parPosVel.position;
            lastPerpPos = (int) perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = (int) (parPosVel.position - lastParPos);
        int perpPosDelta = (int) (perpPosVel.position - lastPerpPos);
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = (int) parPosVel.position;
        lastPerpPos = (int) perpPosVel.position;
        lastHeading = heading;

        return twist;
    }
}
