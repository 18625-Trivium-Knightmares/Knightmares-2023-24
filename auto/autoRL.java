package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.methods;

@Autonomous
public class autoRL extends LinearOpMode {

    /**
     * VARIABLES
     */
    DcMotor FR, FL, BR, BL, AM, HM, CM, SM; // All of the motors
    Servo Claw;

    double openClaw = 0.001;
    double closeClaw = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        methods meth = new methods();

        meth.strafeLeft(500, 0.5);
        meth.verticalMove(5000, 0.5);
    }
}
