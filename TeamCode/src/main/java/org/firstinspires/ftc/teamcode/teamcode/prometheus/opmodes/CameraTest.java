package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Camera;

@TeleOp
public class CameraTest extends LinearOpMode {

    Camera camera;
    ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(this);

        camera.initCamera();

        waitForStart();

        camera.startVuforia();
        camera.startTensorFlow();


        while(opModeIsActive()){
            if (loopTime.milliseconds() > 50) {
                loopTime.reset();
                camera.vuforiaLoop();
                camera.tensorFlowLoop();
                telemetry.update();

                if (gamepad1.x) {
                    camera.servoBack();
                }

                if (gamepad1.y) {
                    camera.servoDown();
                }

                if (gamepad1.b) {
                    camera.servoUp();
                }
            }
        }

        camera.stopVuforia();
        camera.stopTensorFlow();
    }
}
