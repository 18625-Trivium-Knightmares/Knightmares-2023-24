package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous (name = "Red Right")
@Config
public class stAuto extends LinearOpMode {

    /**
     * METHODS
     */

    // START ENCODERS
    public void startEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // EXIT ENCODERS
    public void exitEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // RESTART ENCODERS
    public void resetEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // SET DRIVE POWER
    public void setMotorPower(double power) {
        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);
    }

    public void runEncoders() {
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // DRIVETRAIN MOVEMENT
    public void move(double inches, String direction) {
        resetEncoders();
        startEncoders();
        double newTarget = TPI * inches;

        switch (direction) {
            case "forward":
                FR.setTargetPosition((int) newTarget);
                FL.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) newTarget);
                break;

            case "backward":
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) -newTarget);
                break;

            case "left":
                FR.setTargetPosition((int) newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) newTarget);
                break;

            case "right":
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) -newTarget);
                break;

            case "right_turn":
//                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) newTarget);
//                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) newTarget);
                break;

            case "left_turn":
                FR.setTargetPosition((int) newTarget);
//                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) newTarget);
//                BL.setTargetPosition((int) -newTarget);
                break;
        }
        setMotorPower(1);
        runEncoders();

        while(FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
        }

        resetEncoders();
        exitEncoders();

    }
    /**
     * VARIABLES
     */

    public static double TPI = 45;

    public static double openClaw = 0.001;
    public static double closeClaw = 1;


    public static double strt = 10;
    public static double frstTrn = 10;
    public static double mId = 5;
    public static double midBck = 10;
    public static double normBck = 5;
    public static double backStage = 15;
    public static double end = 5;



    int spikePlacement;
    int rsp;

    // DECLARING MOTORS, SERVOS, AND CAMERA
    OpenCvWebcam webcam = null;
    DcMotor FR, FL, BR, BL, AM, HM, CM, SM;
    Servo Claw;



    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * ASSIGNING MOTORS AND SERVOS
         */

        // Assigning all of the servos, motors, and camera
            //DRIVE TRAIN
        FR = hardwareMap.dcMotor.get("rightFront");
        FL = hardwareMap.dcMotor.get("Buh");
        BR = hardwareMap.dcMotor.get("rightBack");
        BL = hardwareMap.dcMotor.get("Bruh");
            //OTHER
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");
        Claw = hardwareMap.servo.get("Claw");

            // REVERSE THE LEFT DRIVE
        FL.setDirection((DcMotor.Direction.REVERSE));
        BL.setDirection(DcMotor.Direction.REVERSE);

            // CAMERA
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new stAuto.colorPipeline());

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

        Claw.setPosition(openClaw);

        waitForStart();

        rsp = spikePlacement; // tells us where and saves to variable

        webcam.stopStreaming(); // stops camera so that it's set in stone

        move(36, "left");
        move(6, "right");

        /*if(rsp == 2) {
            move(frstTrn, "right_turn");
            move(-frstTrn, "right_turn");
        } else if (rsp == 1) {
            move(mId, "left");
            move(midBck, "right");
        } else if (rsp == 0) {
            move(frstTrn, "left_turn");
            move(-frstTrn, "left_turn");
        }
        move(normBck, "right");
        move(backStage, "forward");
        move(end, "left");*/



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

            leftavgfin = Math.ceil((leftavg.val[0]) + 1);
            midavgfin = Math.ceil((midavg.val[0]));
            rightavgfin = Math.ceil((rightavg.val[0]) + 4);

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
