package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class CrosshairVision {

    public static double fx = 2843.88;
    public static double fy = 2843.88;

    public static double cx = 389.204;
    public static double cy = 262.566;

    public static double objectWidth = 3.5;
    public static double objectHeight = 1.5;

    WebcamName name;
    OpenCvWebcam webcam;

    Mat cameraMatrix;
    MatOfPoint3f objectPoints;
    MatOfDouble distCoeffs;

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

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);

        objectPoints = new MatOfPoint3f(
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0)
        );

        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }

    public RotatedRect getLastRects() {
        return pipeline.getRotRectTarget("sample");
    }

    public Point3 to3d() {
        if(cameraMatrix != null) cameraMatrix.release();

        RotatedRect rect = getLastRects();
        if(rect == null) return null;

        Point[] rectPoints = new Point[4];
        rect.points(rectPoints);

        constructMatrix();

        // You may need to sort these rectPoints to match objectPoints' order
        MatOfPoint2f imgPoints = new MatOfPoint2f(rectPoints);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        boolean success = Calib3d.solvePnP(
                objectPoints,
                imgPoints,
                cameraMatrix,
                distCoeffs,
                rvec, tvec,
                false,
                Calib3d.SOLVEPNP_ITERATIVE
        );

        if(success) {
            double[] coords = new double[3];
            tvec.get(0, 0, coords);

            return new Point3(coords[0], coords[1], coords[2]);
        }

        return null;
    }

    public Point toSimplePoint(double offsetX, double offsetY) {
        RotatedRect rect = getLastRects();

        if(rect == null) {
            return null;
        }

        double centerX = 320 / 2f;
        double deltaX = centerX - rect.center.x;

        double centerY = 0;
        double deltaY = centerY - rect.center.y;

        double width = Math.max(rect.size.width, rect.size.height);

        double factor = CrosshairVision.objectWidth / width;
        return new Point((deltaX * factor) + offsetX, (deltaY * factor) + offsetY);
    }

}