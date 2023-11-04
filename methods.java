package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class methods extends LinearOpMode {
    DcMotor FR, FL, BR, BL, AM, HM, CM, SM; // All of the motors
    Servo Claw;

    public double angle(double ang) {
        return ang / 360;
    }
    public int blocks(int blox) {
        return blox * 24;
    }


    public void strafeLeft(int tIme, double power) {
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(-power);
        FL.setPower(power);
        sleep(tIme);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
    }
    public void strafeRight(int tIme, double power) {
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        sleep(tIme);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
    }
    public void verticalMove(int tIme, double power) {
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        sleep(tIme);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
    }
    public void runOpMode() throws InterruptedException{
    }

}
