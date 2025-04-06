package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PedroSubsystem extends SubsystemBase {

    Follower follower;

    public static Pose EndPose = new Pose();

    public PedroSubsystem(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
        EndPose = follower.getPose();
    }

    public Command fieldCentricCmd(Gamepad gamepad) {
        return new FieldCentricCmd(gamepad);
    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }
    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
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

}