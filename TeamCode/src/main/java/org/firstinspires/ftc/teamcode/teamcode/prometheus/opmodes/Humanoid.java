package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name = "Humanoid")
public class Humanoid extends LinearOpMode {
    public float x, y, z, w, pwr;
    private Servo leftArm;
    private Servo rightArm;
//    boolean opened = false;
//    boolean slowmode = false;

    //private Servo FoundationServo;


    @Override
    public void runOpMode() throws InterruptedException {
//        BR = hardwareMap.dcMotor.get("BR");
//        BL = hardwareMap.dcMotor.get("BL");
//        FL = hardwareMap.dcMotor.get("FL");
//        FR = hardwareMap.dcMotor.get("FR");
//        IR = hardwareMap.dcMotor.get("IR");
//        IL = hardwareMap.dcMotor.get("IL");
//        PulleyL = hardwareMap.dcMotor.get("PulleyL");
//        PulleyR = hardwareMap.dcMotor.get("PulleyR");
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");


//        FR.setDirection(DcMotor.Direction.REVERSE);
//        BR.setDirection(DcMotor.Direction.REVERSE);
//        IL.setDirection(DcMotor.Direction.REVERSE);
//        PulleyL.setDirection(DcMotor.Direction.REVERSE);


//        FR.setPower(Range.clip(pwr - x + z, -1, 1));
//        BL.setPower(Range.clip(pwr - x - z, -1, 1));
//        FL.setPower(Range.clip(pwr + x - z, -1, 1));
//        BR.setPower(Range.clip(pwr + x + z, -1, 1));
//        IL.setPower(Range.clip(pwr + x + z, -.5, .5));
//        IR.setPower(Range.clip(pwr + x + z, -.5, .5));

        waitForStart();
        while (opModeIsActive()) {
// moving the robot forward and backward


            if (gamepad1.dpad_down) {
                leftArm.setPosition(0);
            } else {
                leftArm.setPosition(0.5);
            }

            if (gamepad1.dpad_up) {
                leftArm.setPosition(1);
            } else {
                leftArm.setPosition(0.5);
            }

                if (gamepad1.dpad_right) {
                    rightArm.setPosition(1);
                } else {
                    rightArm.setPosition(0.5);
                }

                    if (gamepad1.dpad_left) {
                        rightArm.setPosition(0);
                    } else {
                        leftArm.setPosition(0.5);
                    }

//            if (gamepad1.dpad_right) {
//                StoneL.setPosition(.1);
//            }
//            if(gamepad1.dpad_left){
//                StoneL.setPosition(.5);
//            }
//            if (gamepad1.x) {
//                FoundationServoL.setPosition(.7);
//                FoundationServoR.setPosition(.3); // down close to the ground
//            }
//            if (gamepad1.y) {
//                FoundationServoL.setPosition(.15); // up on the robot
//                FoundationServoR.setPosition(.85);
//            }
//            if (gamepad1.b){
//                CapstoneServo.setPosition(1);
//            }
//            if (gamepad1.a){
//                CapstoneServo.setPosition(.85);
//            }
//
//
//            IR.setPower(Range.clip(gamepad2.right_trigger, -.7, .7));
//            IL.setPower(Range.clip(gamepad2.right_trigger, -.7, .7) );
//
//            IR.setPower(Range.clip(-gamepad2.left_trigger, -1, 1));
//            IL.setPower(Range.clip(-gamepad2.left_trigger, -1, 1));
//
//            if(opened == false && gamepad2.x){
//
//                OuttakeL.setPosition(1); //takes out of robot
//                OuttakeR.setPosition(.2);
//                sleep(250);
//                opened = true;
//            }
//            if (gamepad2.x && opened == true) {
//                OuttakeL.setPosition(.3); //takes out of robot
//                OuttakeR.setPosition(.8);
//                sleep(250);
//                opened =false;
//            }
//            if(opened == false && gamepad2.a){
//                DepositServo.setPosition(.8);
//                sleep(250);
//                opened = true;
//            }
//            if (gamepad2.a && opened == true) {
//                DepositServo.setPosition(.9);
//                //sleep so the open variable does not get set at the same time as the click
//                sleep(250);
//                opened =false;
//            }
//
//            float forwardBackAxis = gamepad1.left_stick_y; //moving forward
//            float leftRightAxis = -gamepad1.left_stick_x; // strafing
//            float spinAxis = gamepad1.right_stick_x; // turning
//
//
//            //math tings
//            float FRpower = forwardBackAxis + leftRightAxis + spinAxis;
//            float FLpower = forwardBackAxis + leftRightAxis - spinAxis;
//            float BRpower = forwardBackAxis - leftRightAxis + spinAxis;
//            float BLpower = forwardBackAxis - leftRightAxis - spinAxis;
//
//
//            FR.setPower(-FRpower);
//            FL.setPower(-FLpower);
//            BR.setPower(-BRpower);
//            BL.setPower(-BLpower);
//
//            if(gamepad2.b){
//                //close grabber
//                // DepositServo.setPosition(.9);
//                //flip deposit into robot
//                OuttakeL.setPosition(.375); //puts it in
//                OuttakeR.setPosition(.8);
//                sleep(800);
//                //move pulley back to bottom position
//                while(TouchR.getState() == true && TouchL.getState() == true){
//                    PulleyR.setPower(.4);
//                    PulleyL.setPower(-.4);
//                }
//            }
//
//            // if (gamepad2.y){
//            //      OuttakeL.setPosition(.5);
//            //     OuttakeR.setPosition(.6);
//            // }
///*
//            if(opened == false && gamepad2.y){
//                OuttakeL.setPosition(.5);
//                OuttakeR.setPosition(.6);
//                //sleep so the open variable does not get set at the same time as the click
//                sleep(250);
//
//                opened = true;
//            }
//
//            if (gamepad2.a && opened == true) {
//                OuttakeL.setPosition(.3); //takes out of robot
//                OuttakeR.setPosition(.8);
//                //sleep so the open variable does not get set at the same time as the click
//                sleep(250);
//                opened =false;
//            }
//
//*/
//            if(opened == false && gamepad2.y){
//                OuttakeL.setPosition(.6);
//                OuttakeR.setPosition(.6);
//                //sleep so the open variable does not get set at the same time as the click
//                sleep(250);
//
//                opened = true;
//            }
//            if (gamepad2.y && opened == true) {
//                OuttakeL.setPosition(.3);
//                OuttakeR.setPosition(.8);
//                //sleep so the open variable does not get set at the same time as the click
//                sleep(250);
//                opened =false;
//            }
//
//
//            if ((TouchR.getState() == true)) {
//                PulleyR.setDirection(DcMotor.Direction.FORWARD);
//                PulleyR.setPower(gamepad2.left_stick_y);
//                PulleyL.setDirection(DcMotor.Direction.FORWARD);
//                PulleyL.setPower(-gamepad2.left_stick_y);
//
//                //PULLEY.setPower(0);
//                telemetry.addData("Not Pressed", TouchR.getState());
//                telemetry.update();
//            } else {
//                //   telemetry.addData("Pressed", TouchR.getState());
//                PulleyR.setPower(-.3);
//                PulleyL.setPower(.3);
//                telemetry.addData("Not Pressed", TouchR.getState());
//                telemetry.update();
//            }
//
//            if ((TouchL.getState() == true)) {
//                PulleyR.setDirection(DcMotor.Direction.FORWARD);
//                PulleyR.setPower(gamepad2.left_stick_y);
//                PulleyL.setDirection(DcMotor.Direction.FORWARD);
//                PulleyL.setPower(-gamepad2.left_stick_y);
//
//                //PULLEY.setPower(0);
//                telemetry.addData("Not Pressed", TouchL.getState());
//                telemetry.update();
//            } else {
//                //   telemetry.addData("Pressed", TouchR.getState());
//                PulleyR.setPower(-.3);
//                PulleyL.setPower(.3);
//                telemetry.addData("Not Pressed", TouchL.getState());
//                telemetry.update();


                    }
                }
            }


