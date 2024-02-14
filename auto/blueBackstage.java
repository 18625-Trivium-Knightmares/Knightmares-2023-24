package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous (group = "AUTO", name = "BLUE BACKSTAGE")
@Config
//@Disabled
public class blueBackstage extends LinearOpMode {
    /**
     * VARIABLES
     */
    public static double STARTX = 11.67;
    public static double STARTY = 61.5;
    public static double START_HEADING = 180.0;
    public static double LEFT_SPIKEX = 22.0;
    public static double LEFT_SPIKEY = 31.0;
//    public static double LEFT_SPIKE_HEADING = -90.0;
//    public static double LEFT_SPIKE_TANGENT = 180.0;
    public static double LEFT_BACKUPX = 22.0;
    public static double LEFT_BACKUPY = 50.0;
    public static double LEFT_BACKUP_HEADING = 180.0;
    public static double LEFT_BACKDROPX = 58.0;
    public static double LEFT_BACKDROPY = 42.0;
    public static double LEFT_BACKDROP_HEADING = 0.0;
    public static double MID_SPIKEX = 11.67;
    public static double MID_SPIKEY = 20.0;
    public static double MID_SPIKE_HEADING = 180.0;
    public static double MID_BACKUPX = 8.0;
    public static double MID_BACKUPY = 45.0;
    public static double MID_BACKUP_TANGENT = 0.0;
    public static double MID_BACKDROPX = 58.0;
    public static double MID_BACKDROPY = 18.0;
    public static double MID_BACKDROP_TANGENT = 0.0;
    public static double MID_BACKDROP_HEADER = 0.0;

    public static double RIGHT_SPIKEX = 10.0;
    public static double RIGHT_SPIKEY = 25.0;
    public static double RIGHT_SPIKE_HEADER = 90.0;
    public static double RIGHT_SPIKE_TANGENT = 180.0;
    public static double RIGHT_BACKDROPX = 58.0;
    public static double RIGHT_BACKDROPY = 35.0;
    public static double RIGHT_BACKDROP_HEADER = 0.0;

    public static double slowerVelocity = 28.0;


    // declaring motors, servos, and camera
    DcMotor actuator, hoist, chain, slides;
    Servo claw, drone;
    OpenCvWebcam webcam = null;

    public static double blueLeft = -0.5;
    public static double blueMid = 0.0;
    public static double blueRight = 3.0;
    public static double OPEN_CLAW = 0.75;
    public static double CLOSE_CLAW = 0.01;
    public double valLeft;
    public double valMid;
    public double valRight;

    public enum Randomization {
        RIGHT,
        MIDDLE,
        LEFT,
        IDK
    }

    Randomization randomization = Randomization.IDK;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw = hardwareMap.servo.get("Claw");
        chain = hardwareMap.dcMotor.get("ch");

        Trajectory LEFT_SPIKE = drive.trajectoryBuilder(new Pose2d(STARTX, STARTY, Math.toRadians(START_HEADING)))
                .lineToConstantHeading(new Vector2d(LEFT_SPIKEX, LEFT_SPIKEY), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading((new Pose2d(LEFT_BACKUPX, LEFT_BACKUPY, Math.toRadians(LEFT_BACKUP_HEADING))), Math.toRadians(0.0), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory LEFT_END = drive.trajectoryBuilder(LEFT_SPIKE.end())
                .lineToLinearHeading(new Pose2d(LEFT_BACKDROPX, LEFT_BACKDROPY, LEFT_BACKDROP_HEADING), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory MID_SPIKE = drive.trajectoryBuilder(new Pose2d(STARTX, STARTY, Math.toRadians(START_HEADING)))
                .lineToLinearHeading(new Pose2d(MID_SPIKEX, MID_SPIKEY, Math.toRadians(MID_SPIKE_HEADING)), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(MID_BACKUPX, MID_BACKUPY), Math.toRadians(MID_BACKUP_TANGENT), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory MID_END = drive.trajectoryBuilder(MID_SPIKE.end(),  true)
                .splineToLinearHeading(new Pose2d(MID_BACKDROPX, MID_BACKDROPY, Math.toRadians(MID_BACKDROP_HEADER)), Math.toRadians(MID_BACKDROP_TANGENT), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory RIGHT_SPIKE = drive.trajectoryBuilder(new Pose2d(STARTX, STARTY, Math.toRadians(START_HEADING)), true)
                .splineToLinearHeading(new Pose2d(RIGHT_SPIKEX, RIGHT_SPIKEY, Math.toRadians(RIGHT_SPIKE_HEADER)), Math.toRadians(RIGHT_SPIKE_TANGENT), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory RIGHT_END = drive.trajectoryBuilder(RIGHT_SPIKE.end())
                .lineToLinearHeading(new Pose2d(RIGHT_BACKDROPX, RIGHT_BACKDROPY, Math.toRadians(RIGHT_BACKDROP_HEADER)), SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

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

        double ogValLeft = valLeft;
        double ogValMid = valMid;
        double ogValRight = valRight;

        telemetry.addLine("ready to start");
        telemetry.update();

        claw.setPosition(CLOSE_CLAW);

        waitForStart();

        double nwValLeft = valLeft - ogValLeft;
        double nwValMid = valMid - ogValMid + 1;
        double nwValRight = valRight - ogValRight;

        if(nwValRight > nwValLeft && nwValRight > nwValMid ) {
            telemetry.addLine("It is on right");
            randomization = Randomization.RIGHT;
        } else if (nwValLeft > nwValRight && nwValLeft > nwValMid) {
            telemetry.addLine("It is on left");
            randomization = Randomization.LEFT;
        } else if (nwValMid > nwValLeft && nwValMid > nwValRight) {
            telemetry.addLine("It is on middle");
            randomization = Randomization.MIDDLE;
        } else {
            telemetry.addLine("It is on middle for now ig");
            randomization = Randomization.MIDDLE;
        }
        telemetry.addLine("left is: " + String.valueOf(nwValLeft));
        telemetry.addLine("mid is: " + String.valueOf(nwValMid));
        telemetry.addLine("right is: " + String.valueOf(nwValRight));
        telemetry.update();

        webcam.stopStreaming();

        switch (randomization) {
            case RIGHT:
                drive.followTrajectory(RIGHT_SPIKE);
                drive.followTrajectory(RIGHT_END);
                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                chain.setTargetPosition(-950);
                chain.setPower(0.5);
                chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (chain.isBusy()) {
                }
                chain.setPower(0);
                claw.setPosition(OPEN_CLAW);
                break;

            case MIDDLE:
                drive.followTrajectory(MID_SPIKE);
                drive.turn(Math.toRadians(180.0));
                drive.followTrajectory(MID_END);

                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                chain.setTargetPosition(-950);
                chain.setPower(0.5);
                chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (chain.isBusy()) {
                }
                chain.setPower(0);
                claw.setPosition(OPEN_CLAW);
                break;

            case LEFT:
                drive.followTrajectory(LEFT_SPIKE);
                drive.followTrajectory(LEFT_END);

                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                chain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                chain.setTargetPosition(-950);
                chain.setPower(0.5);
                chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (chain.isBusy()) {
                }
                chain.setPower(0);
                claw.setPosition(OPEN_CLAW);
                break;

            case IDK:

                break;

            default:

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

            Rect leftRect = new Rect(1, 178, 150, 120);
            Rect midRec = new Rect(244, 178, 150, 120);
            Rect rightRect = new Rect(489, 178, 150, 120);

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

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            valLeft = leftavgfin;
            valMid = midavgfin;
            valRight = rightavgfin;

            return outPut;
        }
    }
}
