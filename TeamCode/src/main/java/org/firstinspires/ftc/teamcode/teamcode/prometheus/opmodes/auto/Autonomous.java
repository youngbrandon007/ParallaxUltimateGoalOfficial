package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.MotionProfile;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
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

    ElapsedTime loopTime = new ElapsedTime();

    MotionProfile moveProfile = new MotionProfile(30, 30);
    MotionProfile rotProfile = new MotionProfile(2, 2);

    enum program {
        DriveForward,
        Camera,
        DriveToShoot,

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
        if (loopTime.milliseconds() > 50) {
            camera.vuforiaLoop();
            camera.tensorFlowLoop();

            dt.updateTrackerWheels(loopTime.seconds());
            switch (action) {

                case DriveForward:
                    Pos target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(new Pos(0, 0, new Angle()), moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        action = program.Camera;

                    }
                    break;
                case Camera:
                    if (true) {
                        action = program.DriveToShoot;
                    }
                    break;
                case DriveToShoot:

                    break;
            }


            loopTime.reset();
        }

        camera.stopVuforia();
        camera.stopTensorFlow();

    }
    }

}
