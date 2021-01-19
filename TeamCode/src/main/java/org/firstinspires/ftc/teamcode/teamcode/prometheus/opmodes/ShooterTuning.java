package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;


@TeleOp(name = "Shooter PID Tuning")
public class ShooterTuning extends LinearOpMode {

        DriveTrain dt;
        Shooter shooter;

        ElapsedTime time = new ElapsedTime();

        @Override
        public void runOpMode() throws InterruptedException {
                FtcDashboard dashboard = FtcDashboard.getInstance();
                telemetry = dashboard.getTelemetry();

                dt = new DriveTrain(this, true);
                dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                shooter = new Shooter(this);

                waitForStart();
                while(opModeIsActive()){
                        if(time.milliseconds() > 50) {
                                dt.setFromAxis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);


                                if (gamepad1.dpad_up) {
                                        shooter.p += 0.00001;
                                }

                                if (gamepad1.dpad_down) {
                                        shooter.p -= 0.00001;
                                }

                                if (gamepad1.dpad_right) {
                                        shooter.i += 0.000000001; //add zeroes
                                }

                                if (gamepad1.dpad_left) {
                                        shooter.i -= 0.000000001;
                                }
                                if(gamepad1.b){
                                        shooter.d += 0.00001;
                                }
                                if(gamepad1.x){
                                        shooter.d -= 0.00001;
                                }

                                if (gamepad1.right_bumper) {
                                        shooter.target += 10;
                                }

                                if (gamepad1.left_bumper) {
                                        shooter.target -= 10;
                                }
                                if(gamepad1.a){
                                        shooter.target = 0;
                                }
                                if(gamepad1.y){
                                        shooter.target = 1500;
                                }


                                if (gamepad1.right_trigger > 0.1) {
                                        shooter.shooter.setPower(-gamepad1.right_trigger);
                                        shooter.sumError = 0;
                                } else {
                                        shooter.update(time.seconds());
                                }
                                time.reset();
                                telemetry.addData("P Value", shooter.p);
                                telemetry.addData("I Value", shooter.i);
                                telemetry.addData("D Value", shooter.d);
                                telemetry.addData("target Value", shooter.target);
                                telemetry.addData("Target rpm", shooter.target / 28 * 60);
                                telemetry.update();
                        }


                }
        }
}