package org.firstinspires.ftc.teamcode.auto.RedCloseSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Close Side Mid")
public class redSpikeMid extends LinearOpMode {

        // Variables:

        DcMotor FR, FL, BR, BL;
        DcMotor actuator, hoist, chain, slide;
        Servo claw, drone;
        double ticksPerInch = 45.3;


        public void runOpMode() throws InterruptedException {

            // Expansion Hub (DT Motors)
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

            claw.setPosition(0.1);

            useEncoders();
            chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetEncoders();


            // Post Initialization:


            waitForStart();

            moveDriveTrain("backward", 3, 0.3);
            moveDriveTrain("left", 35.5, 0.3);
            moveDriveTrain("right", 3, 0.3);
            moveDriveTrain("forward", 38.5, 0.4);
            moveDriveTrain("left", 4, 0.4);

            chain.setTargetPosition(-550);
            chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chain.setPower(0.2);
            while (chain.isBusy()) {
            }
            chain.setPower(0);

            slide.setTargetPosition(700);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.4);
            while (slide.isBusy()) {
            }
            slide.setPower(0);

            claw.setPosition(0.7);

            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.4);
            while (slide.isBusy()) {
            }
            slide.setPower(0);

            chain.setTargetPosition(0);
            chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chain.setPower(0.2);
            while (chain.isBusy()) {
            }
            chain.setPower(0);


            moveDriveTrain("backward", 0.5, 0.4);
            moveDriveTrain("right", 30, 0.3);
            moveDriveTrain("forward", 7.5, 0.5);

        }

        // METHODS

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

    }

