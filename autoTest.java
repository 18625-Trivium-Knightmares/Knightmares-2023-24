//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Bitmap;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
//import org.opencv.core.Rect;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.openftc.easyopencv.PipelineRecordingParameters;
//
//@Autonomous
//public class autoTest extends LinearOpModeOpMode {
//    OpenCvWebcam webcam = null;
//
//    @Override
//    public void init() {
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "name");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        webcam.setPipeline(new colorPipeline());
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        public void runOpMode() throws InterruptedException {
//
//        }
//    }
//
//    @Override
//    public void loop() {
//
//    }
//
//    class colorPipeline extends OpenCvPipeline {
//        Mat YCbCr = new Mat();
//        Mat leftCrop;
//        Mat rightCrop;
//        double leftavgfin;
//        double rightavgfin;
//        Mat outPut = new Mat();
//        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
//
//        public Mat processFrame(Mat input) {
//            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//            telemetry.addLine("pipeline running");
//
//            Rect leftRect = new Rect(1, 1, 300, 359);
//            Rect rightRect = new Rect(339, 1, 300, 359);
//
//            input.copyTo(outPut);
//            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
//            Imgproc.rectangle(outPut, rightRect, rectColor, 2);
//
//            leftCrop = YCbCr.submat(leftRect);
//            rightCrop = YCbCr.submat(rightRect);
//
//            Core.extractChannel(leftCrop, leftCrop, 2);
//            Core.extractChannel(rightCrop, rightCrop, 2);
//
//            Scalar leftavg = Core.mean(leftCrop);
//            Scalar rightavg = Core.mean(rightCrop);
//
//            leftavgfin = leftavg.val[0];
//            rightavgfin = rightavg.val[0];
//
//            if (leftavgfin > rightavgfin) {
//                telemetry.addLine("It is on the left side");
//            } else if (rightavgfin > leftavgfin) {
//                telemetry.addLine("It is on the right side");
//            } else {
//                telemetry.addLine("It must be in the middle");
//            }
//
//            return outPut;
//        }
//
//    }
//
//}