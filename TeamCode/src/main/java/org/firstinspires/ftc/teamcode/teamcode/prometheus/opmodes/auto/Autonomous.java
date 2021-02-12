package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Camera;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;

public class Autonomous extends LinearOpMode {

    DriveTrain dt;
    Shooter shooter;
    Collector collector;
    WobbleArm wobbleArm;
    Camera camera;

    enum program {
        DriveForward,
        Camera,
        Shoot

    }

    program action = program.DriveForward;

    @Override
    public void runOpMode() throws InterruptedException {

    dt = new DriveTrain(this);
    shooter = new Shooter(this);
    collector = new Collector(this);
    wobbleArm = new WobbleArm(this, true);
    camera = new Camera(this);

    camera.initCamera();
    camera.servoBack();
    dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    waitForStart();

    camera.startVuforia();
    camera.startTensorFlow();
    camera.servoDown();


    while (opModeIsActive()) {
        dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        camera.vuforiaLoop();
        camera.tensorFlowLoop();


        switch(action) {

            case DriveForward:
                if (true) {
                    action = program.Camera;
                }
                break;
            case Camera:
                if (true) {
                    action = program.Shoot;
                }
                break;
            case Shoot:

                break;
            }
        }


        camera.stopVuforia();
        camera.stopTensorFlow();
    }
}
