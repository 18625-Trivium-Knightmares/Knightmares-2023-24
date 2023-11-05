package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.methods;

@Autonomous
public class autoBR extends LinearOpMode {

    /**
     * VARIABLES
     */
    DcMotor FR, FL, BR, BL, AM, HM, CM, SM; // All of the motors
    Servo Claw;

    double openClaw = 0.001;
    double closeClaw = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Assigning all of the servos and motors
        FR = hardwareMap.dcMotor.get("rightFront");
        FL = hardwareMap.dcMotor.get("leftFront");
        BR = hardwareMap.dcMotor.get("rightBack");
        BL = hardwareMap.dcMotor.get("leftBack");
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");

        Claw = hardwareMap.servo.get("Claw");

        waitForStart();

        FR.setDirection((DcMotorSimple.Direction.REVERSE));
        BR.setDirection((DcMotorSimple.Direction.REVERSE));

        FR.setPower(-0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);
        FL.setPower(-0.75);
        sleep(50);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);


        FR.setPower(-0.5);
        BL.setPower(-0.5);
        BR.setPower(-0.5);
        FL.setPower(-0.75);
        sleep(5000);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
    }
}
