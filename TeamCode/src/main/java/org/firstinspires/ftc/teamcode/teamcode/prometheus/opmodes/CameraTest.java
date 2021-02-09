package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Camera;

@TeleOp
public class CameraTest extends LinearOpMode {

    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(this);

        camera.initCamera();

        waitForStart();

        camera.startVuforia();

        while(opModeIsActive()){
            camera.vuforiaLoop();
        }

        camera.stopVuforia();
    }
}
