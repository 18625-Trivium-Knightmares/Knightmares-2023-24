package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Harrison Test Chain")
public class chaintest extends LinearOpMode {

    DcMotor FR, FL, BR, BL;
    DcMotor actuator, hoist, chain, slide;
    Servo claw, drone;
    double ticksPerInch = 45.3;

    @Override
    public void runOpMode() throws InterruptedException {
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

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.6;
            double vertical = -gamepad1.left_stick_y * 0.5;
            double pivot = gamepad1.right_stick_x * 0.5;

            FL.setPower(horizontal + vertical + pivot);
            BL.setPower(-horizontal + vertical + pivot);
            FR.setPower(-horizontal + vertical - pivot);
            BR.setPower(vertical + horizontal - pivot);

            chain.setPower(-gamepad1.right_stick_y * 0.5);

            double slideTicks = slide.getCurrentPosition();
            double chainTicks = chain.getCurrentPosition();

        if (gamepad1.a) {
            slide.setTargetPosition(768);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.4);
            while (slide.isBusy()) {
            }
            slide.setPower(0);
        }

        if (gamepad1.b) {
            slide.setTargetPosition(50);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.4);
            while (slide.isBusy()) {
            }
            slide.setPower(0);
        }


        if (gamepad1.right_bumper) {
            claw.setPosition(0.5);
        } else if (gamepad1.left_bumper) {
            claw.setPosition(0.1);
        }
            telemetry.addData("Slide Ticks", slideTicks);
            telemetry.addData("Chain Ticks", chainTicks);
            telemetry.update();
        }
    }
}
