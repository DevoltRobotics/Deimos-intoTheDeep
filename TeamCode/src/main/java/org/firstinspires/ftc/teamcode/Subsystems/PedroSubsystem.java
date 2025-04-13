package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Constants.getTargetAngleY;
import static org.firstinspires.ftc.teamcode.Subsystems.Constants.xDegreesPerPixel;
import static org.firstinspires.ftc.teamcode.Subsystems.Constants.xFov;
import static org.firstinspires.ftc.teamcode.Subsystems.Constants.yPixels;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config.PIDFController;
import org.firstinspires.ftc.teamcode.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

public class PedroSubsystem extends SubsystemBase {

    public PIDFController chassisController = new PIDFController(Constants.chassisCoefficients);

    public final Follower follower;

    public static Pose EndPose = new Pose();

    DcMotorEx fl, bl, br, fr;

    CrosshairVision vision;

    Telemetry telemetry;

    boolean following = true;

    public PedroSubsystem(Follower follower, CrosshairVision vision, Telemetry telemetry, DcMotorEx fl, DcMotorEx bl, DcMotorEx br, DcMotorEx fr) {
        this.follower = follower;

        this.vision = vision;
        this.telemetry = telemetry;

        this.fl = fl;
        this.bl = bl;
        this.br = br;
        this.fr = fr;
    }

    @Override
    public void periodic() {
        follower.update();

        if (!following) {
            follower.breakFollowing();
        }

        EndPose = follower.getPose();
    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }

    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
    }

    public Command turnChassisVision(double power, IMU imu) {
        return new VisionTurnChassis(power, imu);
    }

    public Command reTurnChassisVision(double power, IMU imu) {
        return new VisionReTurnChassis(power, imu);

    }

    public Command breakPath() {
        return new BreakPath();

    }

    public Command returnPath() {
        return new ReturnPath();
    }


    public Command fieldCentricCmd(Gamepad gamepad) {
        return new FieldCentricCmd(gamepad);
    }


    class FieldCentricCmd extends CommandBase {
        Gamepad gamepad;

        FieldCentricCmd(Gamepad gamepad) {
            this.gamepad = gamepad;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.startTeleopDrive();
        }

        @Override
        public void execute() {
            follower.setTeleOpMovementVectors(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, false);
        }

        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    class FollowPathCmd extends CommandBase {
        Path path;

        FollowPathCmd(Path path) {
            this.path = path;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path);
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    class FollowPathChainCmd extends CommandBase {
        PathChain path;

        FollowPathChainCmd(PathChain path) {
            this.path = path;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path, true);
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    class VisionTurnChassis extends CommandBase {
        boolean targeteado = false;
        double target = 0;
        double power;

        double error;
        double powerMult;
        ElapsedTime timer;

        double size;

        double pixelErrorFromCenterY;

        boolean reajusted = false;

        double angleError;
        double headingDegrees;

        IMU imu;
        public VisionTurnChassis(double power, IMU imu) {
            this.powerMult = power;
            this.imu = imu;

            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();

            following = false;

            RotatedRect rect = vision.getLastRects();

            int targetX;
            int targetY;

            if (rect != null) {
                targetY = (int)rect.center.y;
                targetX = (int)rect.center.x;

                pixelErrorFromCenterY = yPixels - targetY;
                double targetYAngl = getTargetAngleY(targetY);

                double pixelErrorFromCenterX = targetX - 160;
                double targetXAngl = pixelErrorFromCenterX / xDegreesPerPixel;

                double tAngl = targetXAngl + ((targetXAngl/(xFov)) * targetYAngl);

                angleError = Range.clip((tAngl), -15, 15);

                headingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }else {
                target = 5;
                chassisController.targetPosition = target;
                targeteado = true;

            }

            if (!targeteado) {
                target = (int)(headingDegrees - angleError);
                chassisController.targetPosition = target;

                targeteado = true;
            }

            follower.breakFollowing();

        }

        @Override
        public void execute() {
            double headingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = Math.abs(target - headingDegrees);

            power = chassisController.update(headingDegrees) * powerMult;

            if (Math.abs(error) >= 0.4) {
                fl.setPower(-power);
                bl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            }else {
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
            }

            telemetry.addData("heading", headingDegrees);
            telemetry.addData("target", chassisController.targetPosition);
            telemetry.addData("error", error);
        }


        @Override
        public boolean isFinished() {
            return (timer.seconds() >= 0.4) || Math.abs(error) <= 0.4;

        }

        @Override
        public void end(boolean interrupted) {
            /*if (((size <= 4000 && pixelErrorFromCenterY >= 120) ||(size <= 9000 && pixelErrorFromCenterY < 120)) && !reajusted){
                turnChassis(0.5, imu).schedule();
                reajusted = true;
            }else {
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
            }*/

            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

            follower.breakFollowing();
        }
    }

    class VisionReTurnChassis extends CommandBase {
        boolean targeteado = false;
        double target = 0;
        double power;

        double error;
        double powerMult;
        ElapsedTime timer;

        double size;

        double pixelErrorFromCenterY;

        boolean reajusted = false;

        double angleError;
        double headingDegrees;

        IMU imu;

        boolean centered;
        public VisionReTurnChassis(double power, IMU imu) {
            this.powerMult = power;
            this.imu = imu;

            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();

            following = false;

            RotatedRect rect = vision.getLastRects();

            int targetX;
            int targetY;

            if (rect != null) {
                targetY = (int)rect.center.y;
                targetX = (int)rect.center.x;

                centered = Math.abs(targetX) <= 10;

                pixelErrorFromCenterY = yPixels - targetY;
                double targetYAngl = getTargetAngleY(targetY);

                double pixelErrorFromCenterX = targetX - 160;
                double targetXAngl = pixelErrorFromCenterX / xDegreesPerPixel;

                double tAngl = targetXAngl + ((targetXAngl/(xFov)) * targetYAngl);

                angleError = Range.clip((tAngl), -15, 15);

                headingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            }else {
                centered = true;


            }

            if (!targeteado) {
                target = (int)(headingDegrees - angleError);
                chassisController.targetPosition = target;

                targeteado = true;
            }

            follower.breakFollowing();

        }

        @Override
        public void execute() {
            double headingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = Math.abs(target - headingDegrees);

            power = chassisController.update(headingDegrees) * powerMult;

            if (Math.abs(error) >= 0.4) {
                fl.setPower(-power);
                bl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            }else {
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
            }

            telemetry.addData("heading", headingDegrees);
            telemetry.addData("target", chassisController.targetPosition);
            telemetry.addData("error", error);
        }


        @Override
        public boolean isFinished() {
            return (centered) || (timer.seconds() >= 0.4) || Math.abs(error) <= 0.4;
        }

        @Override
        public void end(boolean interrupted) {
            /*if (((size <= 4000 && pixelErrorFromCenterY >= 120) ||(size <= 9000 && pixelErrorFromCenterY < 120)) && !reajusted){
                turnChassis(0.5, imu).schedule();
                reajusted = true;
            }else {
                fl.setPower(0);
                bl.setPower(0);
                fr.setPower(0);
                br.setPower(0);
            }*/

            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
    }

    class HoldPoint extends CommandBase {
        BezierPoint holdPoint;
        double heading;


        public HoldPoint(BezierPoint holdPoint, double heading) {
            this.holdPoint = holdPoint;
            this.heading = heading;
        }

        public HoldPoint(Point holdPoint, double heading) {
            this(new BezierPoint(holdPoint), heading);
        }

        public HoldPoint(Pose holdPose) {
            this(new BezierPoint(new Point(holdPose)), holdPose.getHeading());
        }

        @Override
        public void initialize() {
            follower.holdPoint(holdPoint, heading);
        }

        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }
    }

    class BreakPath extends CommandBase {

        public BreakPath() {
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            following = false;
            follower.breakFollowing();
        }

        @Override
        public boolean isFinished() {
            return true;
        }

    }

    class ReturnPath extends CommandBase {
        public ReturnPath() {
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            following = true;
            follower.resumePathFollowing();
            follower.update();
        }

        @Override
        public void execute() {
            following = true;
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

}