package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class autoTest extends LinearOpMode {
    /**
     * VARIABLES
     */

    // declaring the drive train
    DcMotor FR, FL, BR, BL;
    // declaring webcam
    OpenCvWebcam webcam = null;

    // other variable stuff
    double speed = 1;
    double BRSpeed = 0.75;
    double BLSpeed = 2;

    int spikePlacement;

    int rsp;

    /**
     * METHODS
     */

    // MOVE
    public void move(String direction, double power, int tIme) {
        switch (direction) {
            case "forw":
                FR.setPower(power);
                BR.setPower(power * BRSpeed);
                FL.setPower(power);
                BL.setPower(power * BLSpeed);

                break;

            case "back":
                FR.setPower(-power);
                BR.setPower(-power * BRSpeed);
                FL.setPower(-power);
                BL.setPower(-power * BLSpeed);

                break;

            case "left":
                FR.setPower(power);
                BR.setPower(-power * BRSpeed);
                FL.setPower(-power);
                BL.setPower(power * BLSpeed);

                break;

            case "right":
                FR.setPower(-power);
                BR.setPower(power * BRSpeed);
                FL.setPower(power);
                BL.setPower(-power * BLSpeed);

                break;

            default:
                telemetry.addLine("goofy little baka");
                telemetry.update();

                break;
        }

        sleep(tIme);

        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
    }

    // TURN
    public void turn(int tIme, String direction) {
        if (direction == "right") {
            FL.setPower(0.5);
            BL.setPower(0.5 * BLSpeed);
            FR.setPower(-0.5);
            BR.setPower(-0.5 * BRSpeed);
        } else if (direction == "left") {
            FR.setPower(0.5);
            BR.setPower(0.5 * BRSpeed);
            FL.setPower(-0.5);
            BL.setPower(-0.5 * BLSpeed);
        }

        if (tIme != 0) {
            sleep(tIme);

            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        /**
        * ASSIGNING MOTORS AND SERVOS
        */

        // DRIVE TRAIN
        FR = hardwareMap.dcMotor.get("rightFront");
        FL = hardwareMap.dcMotor.get("Buh");
        BR = hardwareMap.dcMotor.get("rightBack");
        BL = hardwareMap.dcMotor.get("Bruh");

        // REVERSE THE LEFT
        FL.setDirection((DcMotor.Direction.REVERSE));
        BL.setDirection(DcMotor.Direction.REVERSE);

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

        rsp = spikePlacement; // tells us where and saves to variable

        webcam.stopStreaming(); // stops camera so that it's set in stone

        move("right", 0.75, 500);

        if (rsp == 0) {
            telemetry.addLine("we go left");
            telemetry.update();
            turn(2000, "left");
        } else if (rsp == 2) {
            telemetry.addLine("we go right");
            telemetry.update();
            turn(2000, "right");
        } else if (rsp == 1){
            telemetry.addLine("we go middle");
            telemetry.update();
        } else {
            telemetry.addLine("we go middle because there's nothing else and life is meaningless");
            telemetry.update();
        }


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
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(0.0, 0.0, 100.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 250, 359);
            Rect midRec = new Rect(251, 1, 137, 359);
            Rect rightRect = new Rect(389, 1, 250, 359);

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

            leftavgfin = Math.ceil((leftavg.val[0]) - 3.3);
            midavgfin = Math.ceil((midavg.val[0]) + 0.2);
            rightavgfin = Math.ceil((rightavg.val[0]) - 0.5);

            if (rightavgfin > leftavgfin && rightavgfin > midavgfin) {
                telemetry.addLine("It is on the right side");
                spikePlacement = 2;
            } else if (leftavgfin > rightavgfin && leftavgfin > midavgfin) {
                telemetry.addLine("It is on the left side");
                spikePlacement = 0;
            }  else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
                telemetry.addLine("It is in the middle");
                spikePlacement = 1;
            } else {
                telemetry.addLine("I guess we're going for the middle");
                spikePlacement = 1;
            }
            telemetry.addLine("left is: " + String.valueOf(leftavgfin));
            telemetry.addLine("mid is: " + String.valueOf(midavgfin));
            telemetry.addLine("right is: " + String.valueOf(rightavgfin));
            telemetry.update();

            return outPut;
        }

    }

}