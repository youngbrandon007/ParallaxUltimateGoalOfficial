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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends LinearOpMode {

    DriveTrain dt;
    Shooter shooter;
    Collector collector;
    WobbleArm wobbleArm;
    Camera camera;
    ElapsedTime timer = new ElapsedTime();

    ElapsedTime loopTime = new ElapsedTime();

    MotionProfile moveProfile = new MotionProfile(30, 30);
    MotionProfile rotProfile = new MotionProfile(2, 2);

    enum program {
        DriveForward,
        Camera,
        DriveToShoot,
        Shoot1, ShootDrive1, Shoot2, ShootDrive2, Shoot3,
        DriveToWobble1,
        DepositWobble1,
        DriveToWobble2,
        DepositWobble2,

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

    shooter.shooterPusherBack();
    shooter.shooterLiftDown();
    shooter.indexerUp();

    camera.startVuforia();
    camera.startTensorFlow();
    camera.servoDown();

    loopTime.reset();
    dt.resetTrackerWheels();

    Pos target;


    while (opModeIsActive()) {
        if (loopTime.milliseconds() > 50) {
            camera.vuforiaLoop();
            camera.tensorFlowLoop();

            dt.updateTrackerWheels(loopTime.seconds());
            switch (action) {

                case DriveForward:
                    target = (new Pos(24, -16, new Angle(0)));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        timer.reset();
                        camera.noneCounter = 0;
                        camera.singleCounter = 0;
                        camera.quadCounter = 0;

                        action = program.Camera;
                        dt.stop();
                    }
                    break;
                case Camera:
                    if (timer.seconds()>2) {
                        shooter.shooter.setPower(-1);
                        action = program.DriveToShoot;
                    }


                    break;
                case DriveToShoot:
                    target = (new Pos(58, -5, new Angle(0)));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        action = program.Shoot1;
                        timer.reset();
                        shooter.push3.reset();
                        dt.stop();
                    }
                    break;
                case Shoot1:
                    shooter.push1Ring();
                    if (timer.seconds()>1) {
                        action = program.ShootDrive1;
                    }

                    break;
                case ShootDrive1:
                    target = (new Pos(58, 3, new Angle(0)));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        action = program.Shoot2;
                        timer.reset();
                        shooter.push3.reset();
                        dt.stop();
                    }
                    break;
                case Shoot2:
                    shooter.push1Ring();
                    if (timer.seconds()>1) {
                        action = program.ShootDrive2;
                    }
                    break;
                case ShootDrive2:
                    target = (new Pos(58, 11, new Angle(0)));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        action = program.Shoot3;
                        timer.reset();
                        shooter.push3.reset();
                        dt.stop();
                    }
                    break;
                case Shoot3:
                    shooter.push1Ring();
                    if (timer.seconds()>1) {
                        //action = program.DriveToWobble1;
                        shooter.shooter.setPower(0);
                    }
                    break;
                case DriveToWobble1:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        action = program.DepositWobble1;
                        timer.reset();
                        dt.stop();
                    }
                    break;
                case DepositWobble1:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        action = program.DriveToWobble2;
                        timer.reset();
                        dt.stop();
                    }
                    break;
                case DriveToWobble2:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        action = program.DepositWobble2;
                        timer.reset();
                        dt.stop();
                    }
                    break;
                case DepositWobble2:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (target.atPos(dt.trackerWheels.pos, 1, 1)){
                        timer.reset();
                        dt.stop();
                    }
                    break;
            }


            loopTime.reset();
            telemetry.update();
        }

        camera.stopVuforia();
        camera.stopTensorFlow();

    }
    }

}
