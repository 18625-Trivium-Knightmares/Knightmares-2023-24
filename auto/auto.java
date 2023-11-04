//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous
//@Config
//public class auto extends LinearOpMode {
//    DcMotor hoi, ch;
//
//    public static double ticksPerRevHoi = 537.7;
//    public static double integralSumHoi = 0;
//    public static double KpHoi = 0;
//    public static double KiHoi = 0;
//    public static double KdHoi = 0;
//    public static double KfHoi = 0;
//    ElapsedTime timer = new ElapsedTime();
//    private double lastError = 0;
//    public static double ticksPerRevCh = 537.7;
//    public static double integralSumCh = 0;
//    public static double KpCh = 0;
//    public static double KiCh = 0;
//    public static double KdCh = 0;
//    public static double KfCh = 0;
//    ElapsedTime timerCh = new ElapsedTime();
//    private double lastErrorCh = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        hoi = hardwareMap.get(DcMotor.class, "hoist");
//        ch = hardwareMap.get(DcMotor.class, "ch");
//
//        hoi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double power = PIDControlHoi(0.90 * ticksPerRevHoi, hoi.getCurrentPosition());
//            hoi.setPower(power);
//        }
//    }
//
//    public double PIDControlHoi(double reference, double state) {
//        double error = reference - state;
//        integralSumHoi += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * KpHoi) + (derivative * KdHoi) + (integralSumHoi * KiHoi) + (reference * KfHoi);
//        return output;
//    }
//
//    public double PIDControlCh(double reference, double state) {
//        double error = reference - state;
//        integralSumCh += error * timerCh.seconds();
//        double derivative = (error - lastErrorCh) / timerCh.seconds();
//        lastErrorCh = error;
//
//        timerCh.reset();
//
//        double output = (error * KpCh) + (derivative * KdCh) + (integralSumCh * KiCh) + (reference * KfCh);
//        return output;
//    }
//}
