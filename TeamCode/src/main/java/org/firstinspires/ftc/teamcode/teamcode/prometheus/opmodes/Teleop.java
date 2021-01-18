package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;


@TeleOp(name = "lakshmiTeleopTest")
public class Teleop extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;

        ElapsedTime time = new ElapsedTime();

        @Override
        public void runOpMode() throws InterruptedException {
                dt = new DriveTrain(this, true);
                dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                shooter = new Shooter(this);

                waitForStart();
                while(opModeIsActive()){
                        dt.setFromAxis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                        shooter.shooter.setPower(gamepad1.right_trigger);
                        shooter.shooter.setPower(-gamepad1.left_trigger);
<<<<<<< HEAD
=======

                        if(gamepad1.dpad_up){
                                shooter.p += 0.000001;
                        }

                        if(gamepad1.dpad_down){
                                shooter.p -= 0.000001;
                        }

                        if(gamepad1.dpad_right){
                                shooter.i += 0.0000001; //add zeroes
                        }

                        if(gamepad1.dpad_left){
                                shooter.i -= 0.0000001;
                        }

                        if(gamepad1.right_bumper){
                                shooter.target += 10;
                        }

                        if(gamepad1.left_bumper){
                                shooter.target -= 10;
                        }
<<<<<<< HEAD
                        shooter.update(time.seconds());
                        time.reset();
                        telemetry.addData("P Value", shooter.p);
                        telemetry.addData("i Value", shooter.i);
                        telemetry.addData("target Value", shooter.target);
                        telemetry.update();
=======
>>>>>>> 57dabee1679cb06105690ae28cee5f5f25b62cb6
>>>>>>> f3c56d7a3322800cfd7e9738fd0bb5d42a0e9744
                }
        }
}