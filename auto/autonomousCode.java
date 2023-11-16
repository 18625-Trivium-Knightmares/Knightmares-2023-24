package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous

public class autonomousCode extends LinearOpMode{
    // start encoders
    public void startEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // exit encoders
    public void exitEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // restart encoders
    public void resetEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    /**
     *Go forward
     *
     * example: goForward(1000, 0.95) will go forward at 95% power for 1 second
     * example: goForward(0, 0.95) will go forward at 95% power indefinitely
     */
    public void goForward(int tIme, int power) {
        FR.setPower(power);
        FL.setPower(power);
        BR.setPower(power);
        BL.setPower(power);

        if (tIme != 0) { // IF THIS IS SET TO ANY NUMBER OTHER THAN 0 IT WILL MAKE IT SO THAT IT GOES
            sleep(tIme); // FORWARD FOR HOWEVER LONG IT IS SET TO, IF IT'S 0 IT WILL JUST SET
            // THE MOTORS TO THE POWER INDEFINITELY
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }

    /**
     *Go backward
     *
     * example: goBackward(1000, 0.95) will go backward at 95% power for 1 second
     * example: goBackward(0, 0.95) will go backward at 95% power indefinitely
     */
    public void goBackward(int tIme, double power) {
        FR.setPower(-power);
        FL.setPower(-power);
        BR.setPower(-power);
        BL.setPower(-power);

        if (tIme != 0) { // IF THIS IS SET TO ANY NUMBER OTHER THAN 0 IT WILL MAKE IT SO THAT IT GOES
            sleep(tIme); // BACKWARD FOR HOWEVER LONG IT IS SET TO, IF IT'S 0 IT WILL JUST SET
            // THE MOTORS TO THE POWER INDEFINITELY
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }

    /**
     * turning
     *
     * example: turn(2000, "right"); will turn right at 50% power for 2 seconds
     */
    public void turn(int tIme, String direction) {
        if (direction == "right") {
            FL.setPower(0.5);
            BL.setPower(0.5);
            FR.setPower(-0.5);
            BR.setPower(-0.5);
        } else if (direction == "left") {
            FR.setPower(0.5);
            BR.setPower(0.5);
            FL.setPower(-0.5);
            BL.setPower(-0.5);
        }

        if (tIme != 0) { // IF THIS IS SET TO ANY NUMBER OTHER THAN 0 IT WILL MAKE IT SO THAT IT
            sleep(tIme); // TURNS FOR HOWEVER LONG IT IS SET TO, IF IT'S 0 IT WILL JUST SET
            // THE MOTORS TO THE POWER INDEFINITELY
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }

    /**
     * slide right/left
     *
     * example: slide(1500, left); will slide to the left at 50% power for 1.5 seconds
     */
    public void slide(String direction, int targetToPlace) {
        if (direction == "right") {

            startEncoders();

            FL.setTargetPosition(targetToPlace);
            FR.setTargetPosition(targetToPlace);
            BL.setTargetPosition(targetToPlace);
            BR.setTargetPosition(targetToPlace);

            FL.setPower(0.25);
            FR.setPower(-0.25);
            BL.setPower(-0.25);
            BR.setPower(0.25);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        exitEncoders();
        resetEncoders();
        } else if (direction == "left") {

            startEncoders();

            FL.setTargetPosition(targetToPlace);
            FR.setTargetPosition(targetToPlace);
            BL.setTargetPosition(targetToPlace);
            BR.setTargetPosition(targetToPlace);

            FL.setPower(-0.25);
            FR.setPower(0.25);
            BL.setPower(0.25);
            BR.setPower(-0.25);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            exitEncoders();
            resetEncoders();
        }
    }

        /**if (tIme != 0) { // IF THIS IS SET TO ANY NUMBER OTHER THAN 0 IT WILL MAKE IT SO THAT IT
            sleep(tIme); // TURNS FOR HOWEVER LONG IT IS SET TO, IF IT'S 0 IT WILL JUST SET
            // THE MOTORS TO THE POWER INDEFINITELY
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }**/


     // Methods for encoders
     // example for distance:
    //public void convert(double target) {
        //double target = encodersTarget.calculateToPlaceDistance(5);
        //encoders(target);



     // will go forward 5 inches at power 25%

     // example for rotations:
      // double target = encodersTarget.calculateToPlaceRotations(1.5);
      //encoders(target);
     // will go forward 1.5 rotations
     // how many inches you want the thing to go * (537.7/(3.7795276*4))
     public int convert(double inch) {
          inch = inch * (537.7/15.11*1104);
         return((int) inch);
    }
    public void encoders(int targetToPlace) {

        startEncoders();

        FL.setTargetPosition(targetToPlace);
        FR.setTargetPosition(targetToPlace);
        BL.setTargetPosition(targetToPlace);
        BR.setTargetPosition(targetToPlace);

        FL.setPower(0.25);
        FR.setPower(0.25);
        BL.setPower(0.25);
        BR.setPower(0.25);

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    DcMotor FR,FL,BR,BL,AM, HM,CM,SM;
    Servo Claw;
    @Override
    public void runOpMode()throws InterruptedException {
        FR = hardwareMap.dcMotor.get("Front Right");
        FL = hardwareMap.dcMotor.get("Front Left");
        BR = hardwareMap.dcMotor.get("Back Right");
        BL = hardwareMap.dcMotor.get("Back Left");
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");
        Claw = hardwareMap.servo.get("Claw");

        FL.setDirection((DcMotorSimple.Direction.REVERSE));
        BL.setDirection((DcMotorSimple.Direction.REVERSE));
        double openClaw = 0.001;
        double closeClaw = 1;

        Claw.setPosition(closeClaw);

        WebcamName webcam;


        waitForStart();

        int x = convert(22);
        slide("left",x); //Go towards tape
            sleep(1000);

        /** IF YOU NEED TO TURN 90 DEGREES **/
        //turn(2000,"left");
        /**IF YOU NEED TO TURN 180 DEGREES **/
        //turn(5000,"left");
        CM.setPower(-0.2);
                wait(2000);
        Claw.setPosition(openClaw);
                wait(2000);
        /** IF YOU NEED TO TURN 90 DEGREES **/
        //turn(2000,"left");
        /**IF YOU NEED TO TURN 180 DEGREES **/
        //turn(5000,"left");
        }
}




