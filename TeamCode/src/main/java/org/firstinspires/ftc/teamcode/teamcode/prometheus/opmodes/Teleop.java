package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;


@TeleOp(name = "lakshmiTeleopTest")
public class Teleop extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;

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


                        if(gamepad1.dpad_up){
                                shooter.p += 0.001;
                        }

                        if(gamepad1.dpad_down){
                                shooter.p -= 0.001;
                        }

                        if(gamepad1.dpad_right){
                                shooter.i += 0.001;
                        }

                        if(gamepad1.dpad_left){
                                shooter.i -= 0.001;
                        }

                        if(gamepad1.right_bumper){
                                shooter.target += 0.001;
                        }

                        if(gamepad1.left_bumper){
                                shooter.target -= 0.001;
                        }
                }
        }
}