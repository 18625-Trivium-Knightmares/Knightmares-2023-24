package org.firstinspires.ftc.teamcode.extra.RedCloseSide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    @Autonomous(name = "Red Close Auto")
    @Config

    public class autoTest extends LinearOpMode {
        /**
         * VARIABLES
         */
        int choice = 0;
        int rsp;
        DcMotor FR, FL, BR, BL;
        DcMotor actuator, hoist, chain, slide;
        Servo claw, drone;
        double ticksPerInch = 45.3;
        double ticksPerTurn = 407.7;


        public static double redLeft = 0.5;
        public static double redMid = -4;
        public static double redRight = 0;

        // declaring webcam
        OpenCvWebcam webcam = null;

        @Override
        public void runOpMode() throws InterruptedException {

            FR = hardwareMap.get(DcMotor.class, "rightFront");
            FL = hardwareMap.get(DcMotor.class, "leftFront");
            BR = hardwareMap.get(DcMotor.class, "rightBack");
            BL = hardwareMap.get(DcMotor.class, "leftBack");

            // Control Hub
            hoist = hardwareMap.get(DcMotor.class, "hoist");
            chain = hardwareMap.get(DcMotor.class, "ch");
            slide = hardwareMap.get(DcMotor.class, "Slide");
            actuator = hardwareMap.get(DcMotor.class, "actuator");

            claw = hardwareMap.get(Servo.class, "Claw");
            drone = hardwareMap.get(Servo.class, "drones");

            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            BL.setDirection(DcMotorSimple.Direction.REVERSE);

            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            claw.setPosition(0.1);

            useEncoders();
            chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetEncoders();

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
            rsp = choice;
            // RIGHT:
            if (rsp == 1) {
                moveDriveTrain("left", 29.5, 0.2);
                moveDriveTrain("forward", 4.5, 0.2);
                moveDriveTrain("right", 3.1, 0.15);
                sleep(200);
                // Go to Backdrop
                moveDriveTrain("forward", 32, 0.3);

                moveArm();
                moveArmDown();

                moveDriveTrain("backward", 2, 0.5);
                moveDriveTrain("right", 25, 0.3);
                // moveDriveTrain("backward", 24, 0.5);
                moveDriveTrain("forward", 10, 0.5);

                turnDriveTrain("left", 2);
            } else if (rsp == 2) {
                moveDriveTrain("left", 30.5, 0.3);
                moveDriveTrain("backward", 19, 0.1);
                moveDriveTrain("right", 3.5, 0.2);
                moveDriveTrain("forward", 53.5, 0.5);
                moveDriveTrain("left", 14.5, 0.2);
                moveDriveTrain("forward", 1, 0.2);
                // place yellow pixel
                moveArm();
                moveArmDown();
                // PARK
                moveDriveTrain("right", 38, 0.3);
                moveDriveTrain("forward", 7.5, 0.5);
                //  moveDriveTrain("backward", 30, 0.5);

                turnDriveTrain("left", 2);
            } else if (rsp == 3) {
                moveDriveTrain("backward", 4, 0.2);
                moveDriveTrain("left", 33.5, 0.3);
                moveDriveTrain("right", 5, 0.3);
                sleep(200);
                moveDriveTrain("forward", 38, 0.5);
                sleep(200);
                moveDriveTrain("left", 9, 0.4);
                moveDriveTrain("forward", 0.5, 0.2);

                moveArm();
                sleep(500);
                moveDriveTrain("backward", 2, 0.5);
                moveArmDown();

                moveDriveTrain("backward", 0.5, 0.4);
                moveDriveTrain("right", 30, 0.3);
                moveDriveTrain("forward", 7.5, 0.5);
                // moveDriveTrain("backward", 30, 0.5);
                turnDriveTrain("left", 2);
            }

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


            Mat outPut = new Mat();

            Scalar rectColor = new Scalar(230.0, 0.0, 0);


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

                leftavgfin = Math.ceil((leftavg.val[0]));
                midavgfin = Math.ceil((midavg.val[0]) + 2);
                rightavgfin = Math.ceil((rightavg.val[0]) + 5);

                double leftAdd = 0.0;
                double midAdd = Math.round((leftavg.val[0])) - Math.round((midavg.val[0]));
                double rightAdd = Math.round((leftavg.val[0])) - Math.round((rightavg.val[0]));

                if (rightavgfin > leftavgfin && rightavgfin > midavgfin) {
                    telemetry.addLine("It is on the right side");
                    choice = 1;
                } else if (leftavgfin > rightavgfin && leftavgfin > midavgfin) {
                    telemetry.addLine("It is on the left side");
                    choice = 2;
                }  else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
                    telemetry.addLine("It is in the middle");
                    choice = 3;
                } else {
                    telemetry.addLine("I guess we're going for the middle");
                    choice = 0;
                }
                telemetry.addLine("left is: " + String.valueOf(leftavgfin));
                telemetry.addLine("mid is: " + String.valueOf(midavgfin));
                telemetry.addLine("right is: " + String.valueOf(rightavgfin));

                telemetry.addLine("Add " + rightAdd + " to the right for red");
                telemetry.addLine("Add " + midAdd + " to the middle for red");
                telemetry.addLine("Add " + leftAdd + " to the left for red");
                telemetry.addLine("Choice" + choice);
                telemetry.update();


                return outPut;
            }
        }

        public void useEncoders() {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hoist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void resetEncoders() {
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void runToPosition() {
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public void setTarget(double FLpower,  double FRpower , double BLpower, double BRpower) {
            FL.setTargetPosition((int) FLpower);
            FR.setTargetPosition((int) FRpower);
            BL.setTargetPosition((int) BLpower);
            BR.setTargetPosition((int) BRpower);
        }
        public void setDrivePower(double power) {
            FR.setPower(power);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power);
        }

        public void moveDriveTrain(String direction, double inches, double power) {
            double newTarget = inches * ticksPerInch;
            switch (direction) {
                case "forward":
                    setTarget(newTarget, newTarget, newTarget, newTarget);
                    break;
                case "backward":
                    setTarget(-newTarget, -newTarget, -newTarget, -newTarget);
                    break;
                case "left":
                    setTarget(-newTarget, newTarget, newTarget, -newTarget);
                    break;
                case "right":
                    setTarget(newTarget, -newTarget, -newTarget, newTarget);
                    break;
            }
            runToPosition();
            setDrivePower(power);
            while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
            }
            resetEncoders();
            setDrivePower(0);
        }

        public void moveArm() {
            chain.setTargetPosition(-800);
            chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chain.setPower(1);
            while (chain.isBusy()) {
            }

            sleep(500);

            claw.setPosition(0.5);

            sleep(500);

            chain.setPower(0);
        }

        public void moveArmDown() {
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.5);
            while (slide.isBusy()) {
            }
            slide.setPower(0);

            chain.setTargetPosition(-55);
            chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chain.setPower(0.5);
            while (chain.isBusy()) {
            }
            chain.setPower(0);
        }
        public void turnDriveTrain(String direction, double turnNumber) {
            double newTarget = turnNumber * ticksPerTurn;
            if (direction.equals("left")) {
                setTarget(-newTarget, newTarget, -newTarget, newTarget);
            } else if (direction.equals("right")) {
                setTarget(newTarget, -newTarget, newTarget, -newTarget);
            }

            runToPosition();
            setDrivePower(0.5);
            while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
            }
            setDrivePower(0);
        }
        }


