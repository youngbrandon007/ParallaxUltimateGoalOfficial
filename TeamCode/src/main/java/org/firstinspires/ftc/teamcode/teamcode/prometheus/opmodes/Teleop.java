package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Camera;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;
@TeleOp(name = "febTeleop")

public class Teleop extends LinearOpMode {
    DriveTrain dt;
    Shooter shooter;
    Collector collector;
    WobbleArm wobbleArm;
    Camera camera;
    boolean collectorOn = false;
    boolean shooterOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(this);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new Shooter(this);

        collector = new Collector(this);

        wobbleArm = new WobbleArm(this, true);

        camera = new Camera(this);

        waitForStart();

        while (opModeIsActive()) {
            dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //Wobble stuff
            if (gamepad1.dpad_up) {
                wobbleArm.wobbleArm.setPower(1);
            }

            else if (gamepad1.dpad_down) {
                wobbleArm.wobbleArm.setPower(-1);
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

            if (gamepad1.b && !shooter.autoShoot) {
                shooter.autoShoot = true;
                shooter.push3.reset();

            }

            if (shooter.autoShoot) {
                shooter.push3Rings();
            }

            if (gamepad1.right_bumper) {
                collectorOn = true;
                collector.collector.setPower(1);
            }

<<<<<<< HEAD
            if (gamepad1.left_bumper) {
                collectorOn = false;
                collector.collector.setPower(0);
            }

            if (collectorOn) {
                    if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                        collector.collector.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    }

                    else {
                        collector.collector.setPower(1);
                    }
            }

            else {
                collector.collector.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }

            // Shooterupdate function (previous teleop) and turn shooter off 
            if (shooterOn) {

            }

            if (gamepad1.left_bumper) {
                shooterOn = true;
=======
            if (gamepad1.right_trigger > .1){
                if (gamepad1.right_trigger > 0.5) {
                    collector.collector.setPower(gamepad1.right_trigger);
                }
                else {
                    collector.collector.setPower(0);
                }

            }

            if (gamepad1.left_trigger > 0.1){
                if (gamepad1.left_trigger > 0.5){
                    collector.collector.setPower(-gamepad1.left_trigger);
                }
                else {
                    collector.collector.setPower(0);
                }
>>>>>>> 944fc98dbbb4b71329e102467e9b82f095d6a780
            }


        }
    }
}
