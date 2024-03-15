package org.firstinspires.ftc.teamcode.extra;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

@Autonomous (group = "extra", name = "Color Tests (current red-230.0)")
//@Config
@Disabled
public class autoTest extends LinearOpMode {
    /**
     * VARIABLES
     */
    public static double redLeft = 0.5;
    public static double redMid = -4;
    public static double redRight = 0;
    public static double blueLeft = 6;
    public static double blueMid = 1;
    public static double blueRight = 3;

    // declaring webcam
    OpenCvWebcam webcam = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // CAMERA
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new colorPipeline());

        /**
         * FINDING BLOCK
         */

        // opens camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        webcam.stopStreaming();
    }

    /**
     * PIPELINE
     */
    class colorPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();

        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;

        double leftavgfin;
        double midavgfin;
        double rightavgfin;

        /*Mat leftCropB;
        Mat midCropB;
        Mat rightCropB;

        double leftavgfinB;
        double midavgfinB;
        double rightavgfinB;*/

        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(0.0, 0.0, 100.0);
//        Scalar rectColorB = new Scalar(0.0, 0.0, 100.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 119, 150, 120);
            Rect midRec = new Rect(244, 119, 150, 120);
            Rect rightRect = new Rect(489, 119, 150, 120);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, midRec, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRec);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = Math.round((leftavg.val[0]) + blueLeft);
            midavgfin = Math.round((midavg.val[0]) + blueMid);
            rightavgfin = Math.round((rightavg.val[0]) + blueRight);

            double leftAdd = 0.0;
            double midAdd = Math.round((leftavg.val[0])) - Math.round((midavg.val[0]));
            double rightAdd = Math.round((leftavg.val[0])) - Math.round((rightavg.val[0]));

            if (rightavgfin > leftavgfin && rightavgfin > midavgfin) {
                telemetry.addLine("It is on the right side");
            } else if (leftavgfin > rightavgfin && leftavgfin > midavgfin) {
                telemetry.addLine("It is on the left side");
            }  else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
                telemetry.addLine("It is in the middle");
            } else {
                telemetry.addLine("I guess we're going for the middle");
            }
            telemetry.addLine("left is: " + String.valueOf(leftavgfin));
            telemetry.addLine("mid is: " + String.valueOf(midavgfin));
            telemetry.addLine("right is: " + String.valueOf(rightavgfin));

            telemetry.addLine("Add " + rightAdd + " to the right for red");
            telemetry.addLine("Add " + midAdd + " to the middle for red");
            telemetry.addLine("Add " + leftAdd + " to the left for red");
            telemetry.update();


/*

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColorB, 2);
            Imgproc.rectangle(outPut, midRec, rectColorB, 2);
            Imgproc.rectangle(outPut, rightRect, rectColorB, 2);

            leftCropB = YCbCr.submat(leftRect);
            midCropB = YCbCr.submat(midRec);
            rightCropB = YCbCr.submat(rightRect);

            Core.extractChannel(leftCropB, leftCropB, 2);
            Core.extractChannel(midCropB, midCropB, 2);
            Core.extractChannel(rightCropB, rightCropB, 2);

            Scalar leftavgB = Core.mean(leftCropB);
            Scalar midavgB = Core.mean(midCropB);
            Scalar rightavgB = Core.mean(rightCropB);

            leftavgfinB = Math.round((leftavgB.val[0]) + blueLeft);
            midavgfinB = Math.round((midavgB.val[0]) + blueMid);
            rightavgfinB = Math.round((rightavgB.val[0]) + blueRight);

            double leftAddB = 0.0;
            double midAddB = Math.round((leftavgB.val[0])) - Math.round((midavgB.val[0]));
            double rightAddB = Math.round((leftavgB.val[0])) - Math.round((rightavgB.val[0]));

            if (rightavgfinB > leftavgfinB && rightavgfinB > midavgfinB) {
                telemetry.addLine("It is on the right side");
            } else if (leftavgfinB > rightavgfinB && leftavgfinB > midavgfinB) {
                telemetry.addLine("It is on the left side");
            }  else if (midavgfinB > rightavgfinB && midavgfinB > leftavgfinB){
                telemetry.addLine("It is in the middle");
            } else {
                telemetry.addLine("I guess we're going for the middle");
            }
            telemetry.addLine("left is: " + String.valueOf(leftavgfinB));
            telemetry.addLine("mid is: " + String.valueOf(midavgfinB));
            telemetry.addLine("right is: " + String.valueOf(rightavgfinB));

            telemetry.addLine("Add " + rightAddB + " to the left for red");
            telemetry.addLine("Add " + midAddB + " to the left for middle");
            telemetry.addLine("Add " + leftAddB + " to the left for left");
            telemetry.update();*/

            return outPut;
        }
    }
}