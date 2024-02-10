package org.firstinspires.ftc.teamcode.extra;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "extra", name = "failedTele")
//@Disabled
public class TeleOpFieldCentric extends LinearOpMode {
    // Declaring Variables
    DcMotor actuator, slide, chain, hoist;
    Servo claw, drone;
    IMU imu;
    IMU.Parameters myIMUparameters;
    double openClaw = 0.75;
    double closeClaw = 0.01;
    public static double launch = 0.5;
    public static double set = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        // Configuring Electronics;
        // Control Hub Motors
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        // Servos
        claw = hardwareMap.get(Servo.class, "Claw");
        drone = hardwareMap.get(Servo.class, "drones");

        // ENCODERS:
        resetSlide();
        resetChain();
        resetActuator();
        resetHoist();
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            if (gamepad1.dpad_up) {
                setHoistTarget(-214, 0.75);
                setActuatorTarget(6050, 0.65);
            } else if (gamepad1.dpad_down) {
                setActuatorTarget(0, 0.5);
            }

            if (gamepad1.b) {
                drone.setPosition(launch);
            }
            if (gamepad1.x) {
                drone.setPosition(set);
            }

            // GamePad 2 Controls:

            double chainPower = gamepad2.right_stick_y * 0.5;
            double chainTicks = chain.getCurrentPosition();
            double slidePower = -gamepad2.left_stick_y;
            double slideTicks = slide.getCurrentPosition();

            if (chainPower < 0 && chainTicks > -1225) {
                chain.setPower(-0.3);

            } else if (chainPower > 0 && chainTicks < -55) {
                chain.setPower(0.3);
            } else {
                chain.setPower(0);
            }

            if (slidePower > 0 && slideTicks < 1690) {
                slide.setPower(0.4);
            } else if (slidePower < 0 && slideTicks > 0) {
                slide.setPower(-0.4);
            } else {
                slide.setPower(0);
            }
            if (gamepad2.left_bumper) {
                claw.setPosition(closeClaw);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(openClaw);
            }

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
    public void resetChain() {
        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetSlide() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetHoist() {
        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetActuator() {
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setHoistTarget(double targetTicks, double power) {
        hoist.setTargetPosition((int) targetTicks);
        hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoist.setPower(power);
        while (hoist.isBusy()) {
        }
        hoist.setPower(0);
    }
    public void setActuatorTarget(double targetTicks, double power) {
        actuator.setTargetPosition((int) targetTicks);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(power);
        while (actuator.isBusy()) {
        }
        actuator.setPower(0);
    }

}