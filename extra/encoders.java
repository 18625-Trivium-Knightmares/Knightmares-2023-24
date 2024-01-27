package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Encoders Telemetry")
public class encoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        double Actuatorticks = 6050;
        double halfPower = 0.5;
        DcMotor FR, FL, BR, BL, actuator, slide, chain, hoist;
        Servo claw;


        // Expansion Hub:

        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Control Hub:
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        slide = hardwareMap.get(DcMotor.class, "Slide");


        // Servo:

        claw = hardwareMap.servo.get("Claw");

        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Actuator Ticks", actuator.getCurrentPosition());
            telemetry.update();

            FR.setPower(-gamepad1.right_stick_y * halfPower);
            FL.setPower(-gamepad1.left_stick_y * halfPower);
            BR.setPower(-gamepad1.right_stick_y * halfPower);
            BL.setPower(-gamepad1.left_stick_y * halfPower);

            if (gamepad1.left_bumper) {
                FR.setPower(0.5);
                FL.setPower(-0.5);
                BR.setPower(-0.5);
                BL.setPower(0.5);
            } else if (gamepad1.right_bumper) {
                FR.setPower(-0.5);
                FL.setPower(0.5);
                BR.setPower(0.5);
                BL.setPower(-0.5);
            } else {
                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
            }

            if (gamepad1.a) {
                hoist.setTargetPosition((int) 537.7/4);
                hoist.setPower(0.25);
                hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(hoist.isBusy()) {
                }

                hoist.setPower(0);

                actuator.setTargetPosition((int) Actuatorticks);
                actuator.setPower(0.5);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (actuator.isBusy()) {

                }
                actuator.setPower(0);
            }

            if (gamepad1.b) {
                actuator.setTargetPosition(0);
                actuator.setPower(0.3);
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (actuator.isBusy()) {
                }
                actuator.setPower(0);




            }
        }
    }
}