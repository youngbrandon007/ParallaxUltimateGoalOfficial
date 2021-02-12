package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Camera;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;


@TeleOp(name = "Teleop2")
public class TeleOpSecond extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;
        Collector collector;
        WobbleArm wobbleArm;
        Camera camera;

        double collectorSpeed;
        boolean on = false;
        boolean click = false;
        boolean pusherOut = false;
        boolean pusherAuto = false;
        int shoot = 0;

        ElapsedTime time = new ElapsedTime();
        ElapsedTime shoot3 = new ElapsedTime();
        ElapsedTime pusherTimer = new ElapsedTime();


        @Override
        public void runOpMode() throws InterruptedException {
                //telemetry = FtcDashboard.getInstance().getTelemetry();
                dt = new DriveTrain(this);
                dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                shooter = new Shooter(this);

                collector = new Collector(this);

                wobbleArm = new WobbleArm(this, true);

                camera = new Camera(this);

                double currentPos = wobbleArm.wobbleArm.getPower();

                waitForStart();

                time.reset();

                shooter.pusherBack();

                while(opModeIsActive()){
                        if(time.milliseconds() > 50) {
                                dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


                                if (gamepad2.dpad_up && !click) {
                                        click = true;
                                        shooter.target += 25;
                                }

                                if (gamepad2.dpad_down && !click) {
                                        click = true;
                                        shooter.target -= 25;
                                }

                                if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                                        click = false;
                                }

                                if (gamepad2.b) {
                                        on = true;
                                }
                                if (gamepad2.x) {
                                        on = false;
                                }

                                if (gamepad2.right_bumper) {
                                        if(pusherOut){
                                                pusherTimer.reset();
                                                pusherAuto = true;
                                                shooter.pusherBack();
                                        }else {
                                                shooter.indexerOn();
                                                shooter.indexerUp();
                                        }

                                }

                                if(pusherAuto && pusherTimer.seconds() > .3){
                                        shooter.indexerOn();
                                        shooter.indexerUp();
                                        pusherOut = false;
                                }

                                if(pusherAuto && pusherTimer.seconds() > .6){
                                        pusherAuto = false;
                                        shoot = 3;
                                        shoot3.reset();
                                }

                                if (gamepad2.left_bumper) {
                                        shooter.indexerOn();
                                        shooter.indexerDown();
                                        shooter.pusherBack();
                                        pusherOut = false;
                                        on = false;
                                        collectorSpeed = 1;
                                }

                                if (gamepad2.a) {
                                        shooter.indexerOff();
                                }

                                if (gamepad2.y) {
                                        shooter.pusherOut();
                                        pusherOut = true;
                                        on = true;
                                        collectorSpeed = 0;
                                }

                                if(gamepad2.dpad_left){
                                        if(shoot != 1){
                                                shoot = 1;
                                                shoot3.reset();
                                        }
                                }

                                if(shoot > 0){
                                        if(shoot3.seconds() < .075){
                                                shooter.shooterPusherOut();
                                        }else if(shoot3.seconds() > 0.15){
                                                shoot3.reset();
                                                shoot -= 1;
                                        }else{
                                                shooter.shooterPusherBack();
                                        }
                                }

                                if(!shooter.indexerUp) {
                                        if (gamepad1.right_trigger - gamepad1.left_trigger + gamepad2.right_trigger - gamepad2.left_trigger == 0) {
                                                collector.collector.setPower(collectorSpeed);
                                        } else {
                                                collector.collector.setPower(gamepad1.right_trigger - gamepad1.left_trigger + gamepad2.right_trigger - gamepad2.left_trigger);
                                                collectorSpeed = 0;
                                        }
                                }else{
                                        collector.collector.setPower(0);
                                }

                                if (gamepad1.dpad_up) {
                                        wobbleArm.wobbleArm.setPower(-0.4);
                                }
                                else if (gamepad1.dpad_down) {
                                        wobbleArm.wobbleArm.setPower(0.4);
                                }
                                else {
                                        wobbleArm.wobbleArm.setPower(0);
                                }

                                if (gamepad1.dpad_right) {
                                        wobbleArm.servoClose();
                                }

                                if (gamepad1.dpad_left) {
                                        wobbleArm.servoOpen();
                                }

                                if (on) {
                                        //shooter.update(time.seconds());
                                        shooter.shooter.setPower(-1.0);
                                } else {
                                        shooter.shooter.setPower(0);
                                }

                                if(gamepad2.left_stick_y > 0.5){
                                        shooter.shooterLiftDown();
                                }
                                if(gamepad2.left_stick_y < -0.5){
                                        shooter.shooterLiftUp();
                                }

                                if(gamepad2.right_stick_y > .5){
                                        camera.servoDown();
                                }else if(gamepad2.right_stick_y < -.5){
                                        camera.servoUp();
                                }else{
                                        camera.servoBack();
                                }

                                telemetry.addData("Stick", gamepad2.right_stick_y);

                                time.reset();

                                telemetry.addData("target Value", shooter.target);
                                telemetry.update();
                        }
                }
        }
}