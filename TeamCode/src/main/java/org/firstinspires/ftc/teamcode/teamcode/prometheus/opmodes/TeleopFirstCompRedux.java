package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;


@TeleOp(name = "TeleopRedux")
public class TeleopFirstCompRedux extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;
        Collector collector;
        WobbleArm wobbleArm;

        double collectorSpeed;
        boolean on = false;
        boolean click = false;

        ElapsedTime time = new ElapsedTime();
        ElapsedTime shoot3 = new ElapsedTime();

        Rev2mDistanceSensor distanceSensor;

        @Override
        public void runOpMode() throws InterruptedException {
                dt = new DriveTrain(this, true);
                dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                shooter = new Shooter(this);

                collector = new Collector(this);

                wobbleArm = new WobbleArm(this, true);

                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

                waitForStart();

                time.reset();

                while(opModeIsActive()){
                        if(time.milliseconds() > 50) {
                                dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.85);


                                if (gamepad2.dpad_up && !click) {
                                        click = true;
                                        shooter.target += 50;
                                }

                                if (gamepad2.dpad_down && !click) {
                                        click = true;
                                        shooter.target -= 50;
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


                                if (on) {
                                        shooter.update(time.seconds());
                                } else {
                                        shooter.shooter.setPower(0);
                                }

                                if (gamepad1.right_bumper) {
                                        shooter.indexerOn();
                                        shooter.indexerUp();
                                }

                                if (gamepad1.left_bumper) {
                                        shooter.indexerOn();
                                        shooter.indexerDown();
                                        on = false;
                                        collectorSpeed = 1;
                                }

                                if (gamepad2.b) {
                                        shooter.indexerOff();
                                }

                                if (gamepad1.y) {
                                        shooter.pusherOut();
                                        on = true;
                                        collectorSpeed = 0;
                                } else {
                                        shooter.pusherBack();
                                }

                                if (gamepad2.dpad_right || gamepad1.dpad_right) {
                                        shooter.shooterPusherOut();
                                } else {
                                        shooter.shooterPusherBack();
                                }

                                if(gamepad1.dpad_left){
                                        if(shoot3.seconds() < .3){
                                                shooter.shooterPusherOut();
                                        }else if(shoot3.seconds() > 0.6){
                                                shoot3.reset();
                                        }else{
                                                shooter.shooterPusherBack();
                                        }
                                }else{
                                        shoot3.reset();
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

                                if (gamepad1.dpad_down) {
                                        wobbleArm.wobbleServo.setPosition(1);
                                } else if (gamepad1.dpad_up) {
                                        wobbleArm.wobbleServo.setPosition(0);
                                }else {
                                        wobbleArm.wobbleServo.setPosition(0.5);
                                }

                                time.reset();

                                telemetry.addData("target Value", shooter.target);
                                telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                                telemetry.update();
                        }
                }
        }
}