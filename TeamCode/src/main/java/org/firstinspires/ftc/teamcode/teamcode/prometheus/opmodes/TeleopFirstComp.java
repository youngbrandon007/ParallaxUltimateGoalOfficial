package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;

import java.util.Collection;


@TeleOp(name = "Teleop")
public class TeleopFirstComp extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;
        Collector collector;

        double collectorSpeed;
        double speed = 0.7;
        boolean on = false;
        boolean click = false;

        @Override
        public void runOpMode() throws InterruptedException {
                dt = new DriveTrain(this, true);
                dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                shooter = new Shooter(this);

                collector = new Collector(this);



                waitForStart();
                while(opModeIsActive()){
                        dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


                        if(gamepad2.dpad_up && !click){
                                click = true;
                                speed += .05;
                        }

                        if(gamepad2.dpad_down && !click){
                                click = true;
                                speed -= .05;
                        }

                        if(!gamepad2.dpad_up && ! gamepad2.dpad_down){
                                click = false;
                        }

                        if(gamepad2.b){
                                on = true;
                        }
                        if(gamepad2.x ){
                                on = false;
                        }


                        if(on){
                                shooter.shooter.setPower(-speed);
                        }else{
                                shooter.shooter.setPower(0);
                        }

                        if(gamepad2.right_bumper){
                                shooter.indexerOn();
                                shooter.indexerUp();
                        }

                        if(gamepad2.left_bumper){
                                shooter.indexerOn();
                                shooter.indexerDown();
                                on = false;
                                collectorSpeed = 1;
                        }

                        if(gamepad2.b){
                                shooter.indexerOff();
                        }

                        if(gamepad2.y){
                                shooter.pusherOut();
                                on = true;
                                collectorSpeed = 0;
                        }else{
                                shooter.pusherBack();
                        }

                        if(gamepad2.dpad_left || gamepad1.dpad_left){
                                shooter.shooterPusherOut();
                        }else{
                                shooter.shooterPusherBack();
                        }

                        if (gamepad1.right_trigger-gamepad1.left_trigger+gamepad2.right_trigger-gamepad2.left_trigger == 0){
                                collector.collector.setPower(collectorSpeed);
                        }else{
                                collector.collector.setPower(gamepad1.right_trigger-gamepad1.left_trigger+gamepad2.right_trigger-gamepad2.left_trigger);
                                collectorSpeed = 0;
                        }

                        telemetry.addData("Shooter speed", speed);
                        telemetry.update();
                }
        }
}