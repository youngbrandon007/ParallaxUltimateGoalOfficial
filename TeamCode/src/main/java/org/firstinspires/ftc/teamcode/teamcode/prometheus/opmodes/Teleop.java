package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "lakshmiTeleopTest")
public class Teleop extends LinearOpMode {
        // Declare drive motors
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;


        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize drive motors
                frontLeft = hardwareMap.dcMotor.get("frontLeft");
                frontRight = hardwareMap.dcMotor.get("frontRight");
                backLeft = hardwareMap.dcMotor.get("backLeft");
                backRight = hardwareMap.dcMotor.get("backRight");

                // If drive motors are given full power, robot would spin because of the motors being in
                // opposite directions. So reverse one
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                //frontRight.setDirection(DcMotor.Direction.REVERSE);

                // Wait until start button is pressed
                waitForStart();

                // Repeatedly run code in here until stop button is pressed
                while(opModeIsActive()) {

                        //Forward and backward on the right stick
                        frontLeft.setPower(gamepad1.right_stick_y);
                        frontRight.setPower(gamepad1.right_stick_y);
                        backLeft.setPower(gamepad1.right_stick_y);
                        backRight.setPower(gamepad1.right_stick_y);

                        //strafing on the right stick
                        frontLeft.setPower(-gamepad1.right_stick_x);
                        frontRight.setPower(gamepad1.right_stick_x);
                        backLeft.setPower(gamepad1.right_stick_x);
                        backRight.setPower(-gamepad1.right_stick_x);

                        // Give hardware a chance to catch up
                        idle();
                }
        }
}