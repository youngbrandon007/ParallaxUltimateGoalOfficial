package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;

import java.util.Collection;


@TeleOp(name = "TeleopRedux")
public class TeleopFirstCompRedux extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;
        Collector collector;
        WobbleArm wobbleArm;

        double collectorSpeed;
        boolean on = false;
        boolean click = false;
        boolean pusherOut = false;
        boolean pusherAuto = false;
        int shoot = 0;

        ElapsedTime time = new ElapsedTime();
        ElapsedTime shoot3 = new ElapsedTime();
        ElapsedTime pusherTimer = new ElapsedTime();

        Rev2mDistanceSensor distanceSensor;


        @Override
        public void runOpMode() throws InterruptedException {
                telemetry = FtcDashboard.getInstance().getTelemetry();
                dt = new DriveTrain(this, true);
                dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                shooter = new Shooter(this);

                collector = new Collector(this);

                wobbleArm = new WobbleArm(this, true);

                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

                waitForStart();

                time.reset();

                shooter.pusherBack();

                while(opModeIsActive()){
                        if(time.milliseconds() > 50) {
                                dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


/*                                if (gamepad2.dpad_up && !click) {
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

                                if (!gamepad2.y && !gamepad2.dpad_down) {
                                        click = false;
                                }

 */

                                if (gamepad1.b) {
                                        on = true;
                                }
                                if (gamepad1.x) {
                                        on = false;
                                }

                                if (gamepad1.right_bumper) {
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

                                if (gamepad1.left_bumper) {
                                        shooter.indexerOn();
                                        shooter.indexerDown();
                                        shooter.pusherBack();
                                        pusherOut = false;
                                        on = false;
                                        collectorSpeed = 1;
                                }

                                if (gamepad1.a) {
                                        shooter.indexerOff();
                                }

                                if (gamepad1.y || gamepad2.y) {
                                        shooter.pusherOut();
                                        pusherOut = true;
                                        on = true;
                                        collectorSpeed = 0;
                                }


                                if (gamepad1.dpad_right || gamepad1.dpad_right) {
                                        shooter.shooterPusherOut();
                                } else {
                                        shooter.shooterPusherBack();
                                }

                                if(gamepad1.dpad_left){
                                        if(shoot != 1){
                                                shoot = 1;
                                                shoot3.reset();
                                        }
                                }

                                if(shoot > 0){
                                        if(shoot3.seconds() < .3){
                                                shooter.shooterPusherOut();
                                        }else if(shoot3.seconds() > 0.6){
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

                                if (gamepad1.dpad_down || gamepad2.dpad_down) {
                                        wobbleArm.wobbleServo.setPosition(1);
                                } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                                        wobbleArm.wobbleServo.setPosition(0);
                                }else {
                                        wobbleArm.wobbleServo.setPosition(0.5);
                                }

                                if (on) {
                                        shooter.update(time.seconds());
                                } else {
                                        shooter.shooter.setPower(0);
                                }

                                time.reset();

                                telemetry.addData("target Value", shooter.target);
                                telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                                telemetry.update();
                        }
                }
        }
}