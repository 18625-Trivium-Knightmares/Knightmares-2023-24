package org.firstinspires.ftc.teamcode.extra.RedFarSide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Far Side Mid")
public class spikeMid extends LinearOpMode {
    DcMotor FR, FL, BR, BL;
    DcMotor actuator, hoist, chain, slide;
    Servo claw, drone;
    double ticksPerInch = 45.3;
    double ticksPerTurn = 815.4;
    double ticksPerDegree = 9.06;


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

        moveDriveTrain("backward", 6, 0.2);
        strafe("left", 34.5, 0.5);
        strafe("right", 2, 0.2);
        moveDriveTrain("forward", 25, 0.3);
        strafe("left", 1.5, 0.2);
        turnDriveTrain("left", 7);

        sleep(11000);

        moveDriveTrain("forward", 63.5, 0.5);
        sleep(200);
        strafe("left", 5, 0.3);


        moveArm();
        moveDriveTrain("backward", 3, 0.5);
        moveArmDown();
        strafe("left", 23, 0.5);
        turnDriveTrain("left", 90.0);





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
        }
        runToPosition();
        setDrivePower(power);
        while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
        }
        resetEncoders();
        setDrivePower(0);
    }

    public void strafe (String direction,double inches, double power) {
        double newTarget = inches * ticksPerInch;
        switch (direction) {
            case "left":
                setTarget(-newTarget, newTarget, newTarget, -newTarget);
                break;
            case "right":
                setTarget(newTarget, -newTarget, -newTarget, newTarget);
                break;
        }

        FR.setPower(power);
        FL.setPower(power - 0.05);
        BR.setPower(power);
        BL.setPower(power - 0.05);
        runToPosition();

        while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
        }
        resetEncoders();
        setDrivePower(0);
    }

    public void moveArm() {
        // -750 old value
        chain.setTargetPosition(-630);
        chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chain.setPower(0.4);
        while (chain.isBusy()) {
        }

        slide.setTargetPosition(320);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);
        while (slide.isBusy()) {
        }

        moveDriveTrain("forward", 2, 0.2);

        claw.setPosition(0.5);
        sleep(500);

        slide.setPower(0);
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

    public void turnDriveTrain(String direction, double degrees) {
        double newTarget = degrees * ticksPerDegree;
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
        resetEncoders();
    }



}
