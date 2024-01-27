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

                setMotorPower(0.6);
                while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
                    setMotorPower(0.6);
                }
                break;

            case "backward":
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) -newTarget);

                setMotorPower(0.6);
                while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
                    setMotorPower(0.6);
                }
                break;

            case "left":
                FR.setTargetPosition((int) newTarget);
                FL.setTargetPosition((int) -newTarget);
                BR.setTargetPosition((int) -newTarget);
                BL.setTargetPosition((int) newTarget);

                setMotorPower(0.6);
                while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
                    setMotorPower(0.6);
                }
                break;

            case "right":
                FR.setTargetPosition((int) -newTarget);
                FL.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) -newTarget);

                setMotorPower(0.6);
                while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
                    setMotorPower(0.6);
                }
                break;

            case "right_turn":
                FL.setTargetPosition((int) newTarget);
                BL.setTargetPosition((int) newTarget);

                FL.setPower(0.6);
                BL.setPower(0.6 * BLSpeed);

                runEncoders();
                while(FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
                    FL.setPower(0.6);
                    BL.setPower(0.6 * BLSpeed);
                }
                break;

            case "left_turn":
                FR.setTargetPosition((int) newTarget);
                BR.setTargetPosition((int) newTarget);

                FR.setPower(0.6);
                BR.setPower(0.6 * BRSpeed);

                runEncoders();
                while(FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
                    FR.setPower(0.6);
                    BR.setPower(0.6 * BRSpeed);
                }
                break;
        }
        setMotorPower(0);

        resetEncoders();
        exitEncoders();
        sleep(1000);

    }
    /**
     * VARIABLES
     */

    public static double TPI = 45;

    public static double openClaw = 0.001;
    public static double closeClaw = 1;
    public static double speed = 0.90;
    double BRSpeed = 0.75;
    double BLSpeed = 2;


    public static double step1 = 34;

    public static double RLstep1 = 28;
    public static double rTurn1 = 6;
    public static double mid1 = 6;
    public static double step2 = 36;



    int spikePlacement;
    int rsp;

    // DECLARING MOTORS, SERVOS, AND CAMERA
    OpenCvWebcam webcam;
    DcMotor FR, FL, BR, BL, AM, HM, CM, SM;
    Servo Claw;



    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * ASSIGNING MOTORS AND SERVOS
         */

        // Assigning all of the servos, motors, and camera
            //DRIVE TRAIN
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
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

        Claw.setPosition(openClaw);

        waitForStart();

        rsp = spikePlacement; // tells us where and saves to variable

        webcam.stopStreaming(); // stops camera so that it's set in stone


        resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (TPI * RLstep1));
        FL.setTargetPosition((int) (-TPI * RLstep1));
        BR.setTargetPosition((int) (-TPI * RLstep1));
        BL.setTargetPosition((int) (TPI * RLstep1));

        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(speed);

        runEncoders();
        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            FR.setPower(speed);
            FL.setPower(-speed);
            BR.setPower(-speed);
            BL.setPower(speed);
        }
        setMotorPower(0);

        resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (TPI * rTurn1));
        FL.setTargetPosition((int) (-TPI * rTurn1));
        BL.setTargetPosition((int) (TPI * rTurn1));

        FR.setPower(speed);
        BL.setPower(speed);
        FL.setPower(-speed);

        runEncoders();
        while (FL.isBusy() || BL.isBusy()) {
            FR.setPower(speed);
            BL.setPower(speed);
            FL.setPower(-speed);
        }
        setMotorPower(0);


        resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (-TPI * rTurn1));
        FL.setTargetPosition((int) (TPI * rTurn1));
        BL.setTargetPosition((int) (-TPI * rTurn1));

        FR.setPower(-speed);
        BL.setPower(-speed);
        FL.setPower(speed);

        runEncoders();
        while (FL.isBusy() || BL.isBusy()) {
            FR.setPower(-speed);
            BL.setPower(-speed);
            FL.setPower(speed);
        }
        setMotorPower(0);

        resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (-TPI * step2));
        FL.setTargetPosition((int) (-TPI * step2));
        BR.setTargetPosition((int) (-TPI * step2));
        BL.setTargetPosition((int) (-TPI * step2));

        FR.setPower(-speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(-speed);

        runEncoders();
        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            FR.setPower(-speed);
            FL.setPower(-speed);
            BR.setPower(-speed);
            BL.setPower(-speed);
        }
        setMotorPower(0);



/*        move(step1, "left");
        move(mid1, "left");
        move(mid1, "right");
        move(step2, "backward");
*/

        /*if(rsp == 2) {
            move(frstTrn, "right_turn");
            move(-frstTrn, "right_turn");
        } else if (rsp == 1) {
            resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (TPI * step1));
        FL.setTargetPosition((int) (-TPI * step1));
        BR.setTargetPosition((int) (-TPI * step1));
        BL.setTargetPosition((int) (TPI * step1));

        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(speed);

        runEncoders();
        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            FR.setPower(speed);
            FL.setPower(-speed);
            BR.setPower(-speed);
            BL.setPower(speed);
        }
        setMotorPower(0);

        resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (-TPI * mid1));
        FL.setTargetPosition((int) (TPI * mid1));
        BR.setTargetPosition((int) (TPI * mid1));
        BL.setTargetPosition((int) (-TPI * mid1));

        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(-speed);

        runEncoders();
        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            FR.setPower(-speed);
            FL.setPower(speed);
            BR.setPower(speed);
            BL.setPower(-speed);
        }
        setMotorPower(0);

        resetEncoders();
        startEncoders();

        FR.setTargetPosition((int) (-TPI * step2));
        FL.setTargetPosition((int) (-TPI * step2));
        BR.setTargetPosition((int) (-TPI * step2));
        BL.setTargetPosition((int) (-TPI * step2));

        FR.setPower(-speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(-speed);

        runEncoders();
        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            FR.setPower(-speed);
            FL.setPower(-speed);
            BR.setPower(-speed);
            BL.setPower(-speed);
        }
        setMotorPower(0);
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

            leftavgfin = Math.ceil((leftavg.val[0]));
            midavgfin = Math.ceil((midavg.val[0]) + 1);
            rightavgfin = Math.ceil((rightavg.val[0]) + 6);

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
