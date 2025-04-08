package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class CrosshairVision {

    WebcamName name;
    OpenCvWebcam webcam;

    public Crosshair_Example pipeline = new Crosshair_Example();

    public CrosshairVision(WebcamName name) {
        this.name = name;
    }

    public void init() {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(name);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN, OpenCvWebcam.StreamFormat.MJPEG);

                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void updateExposure() {
    }

    public RotatedRect[] getLastRects() {
        return pipeline.getLastRects();
    }

}