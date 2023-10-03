package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class test extends LinearOpMode {
    DcMotor FR, FL, BR, BL, A, H;

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.get(DcMotorEx.class, "Front Right");
        FL = hardwareMap.get(DcMotorEx.class, "Front Left");
        BR = hardwareMap.get(DcMotorEx.class, "Back Right");
        BL = hardwareMap.get(DcMotorEx.class, "Back Left");

        BR.setDirection((DcMotor.Direction.REVERSE));
        FR.setDirection((DcMotor.Direction.REVERSE));

        A = hardwareMap.get(DcMotorEx.class, "Linear Actuator");
        H = hardwareMap.get(DcMotorEx.class, "Hoist Arm");


        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                A.setPower(0.5);
                sleep(100);
                A.setPower(0);
            }

            if (gamepad1.b) {
                A.setPower(-0.5);
                sleep(100);
                A.setPower(0);
            }
            if (gamepad1.dpad_down) {
                H.setPower(0.5);
                sleep(100);
                H.setPower(0);
            }

            if (gamepad1.dpad_up) {
                H.setPower(-0.5);
                sleep(100);
                H.setPower(0);
            }
        }
    }

}
