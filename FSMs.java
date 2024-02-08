package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(group = "extra", name = "FSMs practice")
@Config
//@Disabled
public class FSMs extends LinearOpMode {
    /**
     * VARIABLES
     */

    // DECLARING MOTORS, SERVOS, AND IMU
    DcMotor FR, FL, BR, BL, actuator, slide, chain, hoist;
    Servo claw, drone;
    IMU imu;
    IMU.Parameters myIMUparameters;


    // OTHER VARIABLES
    public static double SLOWMODE = 0.5;
    public static double OPEN_CLAW = 0.01;
    public static double CLOSE_CLAW = 0.75;
    public static double LAUNCH = 0.5;
    public static double SET = 0.01;
    public static double ACTUATOR_HIGHT = 6050.0;
    public static double HOIST_HIGHT = -214.0;
    public static double ARM_DOWN = -1225.0;
    public static double ARM_UP = -55.0;
    public static double ARM_OUT = 1690.0;


    public enum RigState {
        RIG_START,
        RIG_ARM_UP,
        RIG_DRONE,
        RIG_READY,
        RIG_DONE
    }
    RigState rigState = RigState.RIG_START;

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * ASSIGNING EVERYTHING (MOTORS AND SERVOS)
         */
        // Extension Hub
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        // Control Hub
        hoist = hardwareMap.get(DcMotor.class, "hoist");
        chain = hardwareMap.get(DcMotor.class, "ch");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        // Servos
        claw = hardwareMap.get(Servo.class, "Claw");
        drone = hardwareMap.get(Servo.class, "drones");

        // ENCODERS:
        resetSlide();
        resetChain();
        resetActuator();
        resetHoist();

        // IMU
        imu = hardwareMap.get(IMU.class, "imu"); // Initializing IMU in Drivers Hub
        // Reconfiguring IMU orientation
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        telemetry.addData("NOTE", "Make sure to reset the positions of the chain, actuator, hoist, and slides!");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double vertical = -gamepad1.left_stick_y * 1;
                double horizontal = gamepad1.left_stick_x * 1;
                double pivot = gamepad1.right_stick_x * 1;

                if (gamepad1.right_trigger > 0) {
                    vertical = -gamepad1.left_stick_y * SLOWMODE;
                    horizontal = gamepad1.left_stick_x * SLOWMODE;
                    pivot = gamepad1.right_stick_x * SLOWMODE;
                }

                // Kinematics (Counter-acting angle of robot's heading)
                double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
                double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

                // Setting Field Centric Drive
                FL.setPower(newVertical + newHorizontal + pivot);
                FR.setPower(newVertical - newHorizontal - pivot);
                BL.setPower(newVertical - newHorizontal + pivot);
                BR.setPower(newVertical + newHorizontal - pivot);

                // Resetting "Forwards" Configuration
                if (gamepad1.start) {
                    imu.resetYaw();
                }
                if (gamepad1.x) {
                    drone.setPosition(SET);
                }

                if (gamepad1.dpad_left && rigState != RigState.RIG_START) {
                    rigState = RigState.RIG_START;
                }

                switch (rigState) {
                    case RIG_START:
                        if (gamepad1.dpad_up) {
                            setActuatorTarget(ACTUATOR_HIGHT, 0.65);
                            rigState = RigState.RIG_ARM_UP;
                        }
                        if (gamepad1.y)
                        {
                            setActuatorTarget(0, 0.65);
                            setHoistTarget(0, 0.75);

                        }
                        break;

                    case RIG_ARM_UP:
                        if (actuator.getCurrentPosition() <= ACTUATOR_HIGHT + 1 || actuator.getCurrentPosition() >= ACTUATOR_HIGHT - 1) {
                            drone.setPosition(LAUNCH);
                            rigState = RigState.RIG_DRONE;
                        }
                        break;

                    case RIG_DRONE:
                        if (drone.getPosition() == LAUNCH) {
                            setHoistTarget(HOIST_HIGHT, 0.75);
                            rigState = RigState.RIG_READY;
                        }
                        break;

                    case RIG_READY:
                        if (gamepad1.dpad_down) {
                            setActuatorTarget(0, 0.5);
                            rigState = RigState.RIG_DONE;
                        }

                    default:
                        rigState = RigState.RIG_START;
                }

                // GamePad 2 Controls:

                double chainPower = gamepad2.right_stick_y * 0.5;
                double chainTicks = chain.getCurrentPosition();
                double slidePower = -gamepad2.left_stick_y;
                double slideTicks = slide.getCurrentPosition();


                if (chainPower < 0 && chainTicks > ARM_DOWN) {
                    chain.setPower(-0.3);

                } else if (chainPower > 0 && chainTicks < ARM_UP) {
                    chain.setPower(0.3);
                } else {
                    chain.setPower(0);
                }

                if (slidePower > 0 && slideTicks < ARM_OUT) {
                    slide.setPower(0.4);
                } else if (slidePower < 0 && slideTicks > 0) {
                    slide.setPower(-0.4);
                } else {
                    slide.setPower(0);
                }


                if (gamepad2.left_bumper) {
                    claw.setPosition(OPEN_CLAW);
                } else if (gamepad2.right_bumper) {
                    claw.setPosition(CLOSE_CLAW);
                }
            }
        }

    }

    /**
     * METHODS
     */
    public void resetChain() {
        chain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetSlide() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetHoist() {
        hoist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetActuator() {
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setHoistTarget(double targetTicks, double power) {
        hoist.setTargetPosition((int) targetTicks);
        hoist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoist.setPower(power);
        while (hoist.isBusy()) {
        }
        hoist.setPower(0);
    }
    public void setActuatorTarget(double targetTicks, double power) {
        actuator.setTargetPosition((int) targetTicks);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        actuator.setPower(power);
        while (actuator.isBusy()) {
        }
        actuator.setPower(0);
    }

}