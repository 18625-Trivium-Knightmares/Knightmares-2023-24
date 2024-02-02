package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (group = "realTele", name = "TeleOp-ROBOTC")
//@Disabled
public class teleOpCode extends LinearOpMode {
    /**
     * ALL THE VARIABLES
     */

    DcMotor AM, HM, CM, SM; // All of the motors
    public static double ticks = 384.5;
    public static double ticks2 = 444;
    public static double newTarget;
    public static double launch = 0.5;
    public static double set = 0.01;
    Servo Claw, drone;

    public static double BRSpeed = 0.75;
    public static double BLSpeed = 2;
    public static double speed = 1;
    public static double halfspeed = 0.5; //This was for the faster moving parts (Chain motor)
    public static double openClaw = 0.001;
    public static double closeClaw = 1;

    public static double ticksHar = 537.7;
    public static double turn = ticksHar / 2;
    public static double Actuatorticks = 6050;

    @Override
    public void runOpMode() throws InterruptedException {
        // Assigning all of the servos and motors
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");
        Claw = hardwareMap.servo.get("Claw");
        drone = hardwareMap.servo.get("drones");

        telemetry.addData(">", "Press Play to start op mode"); // Will add stuff to the driver hub screen
        telemetry.update(); // Will update the driver hub screen so that the above will appear

        waitForStart(); // When the start button is pressed

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                /**
                 * ALL UNDER GAMEPAD 1
                 */

                // Drive Train
                if (gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();

                    Pose2d poseEstimate = drive.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();
                } else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0,
                                    0
                            )
                    );
                }


                if (gamepad1.dpad_up) {
                    HM.setTargetPosition((int) (-537.7/2.5));
                    AM.setTargetPosition((int) Actuatorticks);
                    HM.setPower(0.5);
                    AM.setPower(1);
                    HM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while(HM.isBusy() || AM.isBusy()) {
                    }

                    HM.setPower(0);
                    AM.setPower(0);
                    telemetry.addLine("actuator is " + AM.getCurrentPosition());
                    telemetry.addLine("hook is " + HM.getCurrentPosition());
                    telemetry.update();

                } else if (gamepad1.dpad_down) {
                    AM.setTargetPosition(0);
                    AM.setPower(1);
                    AM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (AM.isBusy()) {
                    }
                    AM.setPower(0);
                } else {
                    AM.setPower(0);
                    HM.setPower(0);
                }


                if (gamepad1.b) {
                    drone.setPosition(launch);
                }
                if (gamepad1.x) {
                    drone.setPosition(set);
                }


                /**
                 * ALL UNDER GAMEPAD 2
                 */

                CM.setPower(gamepad2.right_stick_y * halfspeed); //THESE ARE THE OLD GAMEPAD 2 CONTROLS

                SM.setPower(gamepad2.left_stick_y * -speed); //THESE ARE THE  OLD GAMEPAD 2 CONTROLS

                if (gamepad1.y) {
                    SM.setPower(speed);
                } else if (gamepad1.a) {
                    SM.setPower(-speed);
                } else {
                    SM.setPower(0);
                }

                if (gamepad2.left_bumper) {         //gamepad1.left_trigger > 0 OLD GAMEPAD 2 CONTROLS
                    Claw.setPosition(openClaw);
                }
                if (gamepad2.right_bumper) {       //gamepad1.right_bumper > 0  OLD GAMEPAD 2 CONTROLS
                    Claw.setPosition(closeClaw);
                }
            }
        }
    }
}



