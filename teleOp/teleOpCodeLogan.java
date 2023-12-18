package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tank Drive")

public class teleOpCodeLogan extends LinearOpMode {
    /**
     * ALL THE VARIABLES
     */

    DcMotor FR, FL, BR, BL, AM, HM, CM, SM; // All of the motors
    double ticks = 384.5;
    double ticks2 = 444;
    double newTarget;
    Servo Claw;


    double speed = 1;
    double halfspeed = 0.5; //This was for the faster moving parts (Chain motor)
    double openClaw = 0.001;
    double closeClaw = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        // Assigning all of the servos and motors
        FR = hardwareMap.dcMotor.get("rightFront");
        FL = hardwareMap.dcMotor.get("Buh");
        BR = hardwareMap.dcMotor.get("rightBack");
        BL = hardwareMap.dcMotor.get("Bruh");
        CM = hardwareMap.dcMotor.get("ch");
        HM = hardwareMap.dcMotor.get("hoist");
        AM = hardwareMap.dcMotor.get("actuator");
        SM = hardwareMap.dcMotor.get("Slide");
        Claw = hardwareMap.servo.get("Claw");

        FR.setDirection((DcMotorSimple.Direction.REVERSE));
        //BR.setDirection((DcMotorSimple.Direction.REVERSE));

        telemetry.addData(">", "Press Play to start op mode"); // Will add stuff to the driver hub screen
        telemetry.update(); // Will update the driver hub screen so that the above will appear
//        CM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        SM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart(); // When the start button is pressed


            while (opModeIsActive()) {

                /**
                 * ALL UNDER GAMEPAD 1
                 */
                // Basic robot moving controls
                FR.setPower(gamepad1.right_stick_y * speed);
                BR.setPower(gamepad1.right_stick_y * (speed*0.75));
                FL.setPower(gamepad1.left_stick_y * speed);
                BL.setPower(gamepad1.left_stick_y * (speed*2));

                if (gamepad1.dpad_up) {
                    AM.setPower(1);
                } else if (gamepad1.dpad_down) {
                    AM.setPower(-1);
                } else {
                    AM.setPower(0);
                }
                if (gamepad1.dpad_left) {
                    HM.setPower(0.15);
                } else if (gamepad1.dpad_right) {
                    HM.setPower(-0.15);
                } else {
                    HM.setPower(0);
                }
                //This is for strafing'
                //THESE ARE THE OLD GAMEPAD 2 CONTROLS
                if (gamepad1.right_trigger > 0) {
                    FR.setPower(speed);
                    BL.setPower(speed*2);
                    BR.setPower(-speed*0.75);
                    FL.setPower(-speed);
                } else {
                    if (gamepad1.left_trigger > 0) {
                        FR.setPower(-speed);
                        BL.setPower(-speed*2);
                        BR.setPower(speed*0.75);
                        FL.setPower(speed);
                    } else {
                        FR.setPower(0);
                        BL.setPower(0);
                        BR.setPower(0);
                        FL.setPower(0);
                    }
                }
                CM.setPower(gamepad2.right_stick_y*halfspeed); //THESE ARE THE OLD GAMEPAD 2 CONTROLS
                /**
                 * ALL UNDER GAMEPAD 2
                 */
                /**if (gamepad1.b) {
                    CM.setPower(halfspeed);
                } else if (gamepad1.x) {
                    CM.setPower(-halfspeed);
                } else {
                    CM.setPower(0);
                }**/
                SM.setPower(gamepad2.left_stick_y * -speed); //THESE ARE THE  OLD GAMEPAD 2 CONTROLS
                /**if (gamepad1.y) {
                    encoder(2);
//                    SM.setPower(speed);
                }

                if (gamepad1.a) {
                    SM.setPower(-speed);
                } else {
                    SM.setPower(0);
                }**/
                if (gamepad2.left_bumper) {         //gamepad1.left_trigger > 0 OLD GAMEPAD 2 CONTROLS
                    Claw.setPosition(openClaw);
                }
                if (gamepad2.right_bumper) {       //gamepad1.right_bumper > 0  OLD GAMEPAD 2 CONTROLS
                    Claw.setPosition(closeClaw);
                }
            }
        }

    public void encoder(int a){
//        SM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        newTarget = ticks/a;
        SM.setTargetPosition((int)newTarget);
        SM.setPower(speed);
        SM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(SM.isBusy()) {
        }
        SM.setPower(0);
        SM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoder2(int a){
        newTarget = ticks2/a;
        CM.setTargetPosition((int)newTarget);
        CM.setPower(speed);
        CM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(CM.isBusy()){
        }
        CM.setPower(0);
        CM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
//CM ENCODER!!!!!!!!!
