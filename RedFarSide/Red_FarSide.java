package org.firstinspires.ftc.teamcode.auto.RedFarSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

public class Red_FarSide extends LinearOpMode {
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

        useEncoders();
        resetEncoders();
        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw.setPosition(0.1);

        // Post Initialization:
        // Mid:

        waitForStart();
        /*
        moveDriveTrain("backward", 3, 0.3);
        moveDriveTrain("left", 34.5, 0.3);
        moveDriveTrain("right", 3, 0.3);
        */

        // Left:
        moveDriveTrain("backward", 14.5, 0.2);
        moveDriveTrain("left", 33.5, 0.3);
        moveDriveTrain("right", 1.5, 0.3);
        moveDriveTrain("forward", 10, 0.5);
        moveDriveTrain("left", 35, 0.5);
        moveDriveTrain("forward", 90, 0.5);


        /*
        moveDriveTrain("backward", 3, 0.3);
        moveDriveTrain("left", 34, 0.3);
        moveDriveTrain("right", 5, 0.3);
        moveDriveTrain("backward", 15, 0.5);
        moveDriveTrain("left", 37, 0.3);
        moveDriveTrain("forward", 80, 0.8);
        moveDriveTrain("right", 27, 0.3);
        moveDriveTrain("forward", 20, 0.5);

        chain.setTargetPosition((int) -550);
        slide.setTargetPosition((int) 750);
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chain.setPower(0.4);
        sleep(1000);
        slide.setPower(0.5);
        while (chain.isBusy() && slide.isBusy()) {
        }
        chain.setPower(0);
        slide.setPower(0);

        claw.setPosition(0.5);
*/

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

