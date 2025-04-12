package org.firstinspires.ftc.teamcode.Commands.Pedro;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.Vision.CrosshairVision;
import org.opencv.core.Point;

public class PedroVisionAlignCmd extends CommandBase {

    PedroSubsystem subsystem;
    CrosshairVision vision;

    Telemetry telemetry;

    public PedroVisionAlignCmd(PedroSubsystem subsystem, CrosshairVision vision, Telemetry telemetry) {
        this.subsystem = subsystem;
        this.vision = vision;

        this.telemetry = telemetry;
    }

    Point point;
    Pose targetPos;

    @Override
    public void execute() {
        if(point == null) {
            point = vision.toSimplePoint(0, 0);
        } else {
            if(targetPos == null) {
                Pose objPose = new Pose(point.x, point.y);

                Pose pose = subsystem.follower.getPose().copy();
                Vector objVector = objPose.getVector();
                // objVector.rotateVector(pose.getHeading());

                pose.add(new Pose(objVector.getXComponent(), objVector.getYComponent()));

                targetPos = pose;
            }

            subsystem.follower.holdPoint(targetPos);
            telemetry.addData("point", point);
        }
    }

}
