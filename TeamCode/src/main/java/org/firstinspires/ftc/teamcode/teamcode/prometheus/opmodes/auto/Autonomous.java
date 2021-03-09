package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
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

import static org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.SaveGetFile.savePosition;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous extends LinearOpMode {

    DriveTrain dt;
    Shooter shooter;
    Collector collector;
    WobbleArm wobbleArm;
    Camera camera;
    ElapsedTime timer = new ElapsedTime();

    ElapsedTime loopTime = new ElapsedTime();

    MotionProfile moveProfile = new MotionProfile(100, 120);
    MotionProfile rotProfile = new MotionProfile(3, 4);

    enum program {
        DriveForward,
        Camera,
        DriveToShoot,
        Shoot1, ShootDrive1, Shoot2, ShootDrive2, Shoot3,
        DriveToWobble1,
        Wobble1Arm,
        WaitAfterWobble1,
        DepositWobble1,
        DriveToWobble2,
        CloseGrabber,
        PrepWobble2,
        DepositWobble2,
        WaitAfterWobble2,
        Finish,
        Stop
    }

    program action = program.DriveForward;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        dt = new DriveTrain(this);
        shooter = new Shooter(this);
        collector = new Collector(this);
        wobbleArm = new WobbleArm(this, true);
        camera = new Camera(this);

        camera.initCamera();
        camera.servoBack();
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.shooterPusherOut();
        shooter.pusherBack();
        shooter.shooterLiftDown();
        shooter.indexerAutoInit();


        waitForStart();

        wobbleArm.servoClose();

        shooter.shooterLiftDown();

        //camera.startVuforia();
        camera.startTensorFlow();
        camera.servoDown();

        loopTime.reset();

        dt.resetTrackerWheels();
        dt.trackerWheels.updateAngle();
        dt.trackerWheels.robotAngle = new Angle();
        dt.trackerWheels.pos = new Pos();
        dt.trackerWheels.oldPos = new Pos();

        Pos target;

        int rings = 0;

        //action = program.Wobble1Arm;

        wobbleArm.wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {
            if (loopTime.milliseconds() > 50) {
                shooter.indexerUp();

                camera.vuforiaLoop();
                //camera.tensorFlowLoop();


                dt.updateTrackerWheels(loopTime.seconds());
                switch (action) {

                    case DriveForward:
                        target = (new Pos(18, -8, new Angle().setDegrees(-20)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                            timer.reset();
                            camera.noneCounter = 0;
                            camera.singleCounter = 0;
                            camera.quadCounter = 0;

                            action = program.Camera;
                            dt.stop();
                        }
                        break;
                    case Camera:
                        shooter.target = shooter.ShooterPowerSpeed;//shooter.setPower(a crap ton);
                        shooter.indexerUp();
                        shooter.shooterPusherBack();
                        if (timer.seconds() > 2) {

                            action = program.DriveToShoot;
                            if (camera.noneCounter > camera.singleCounter && camera.noneCounter > camera.quadCounter) {
                                rings = 0;
                            } else if (camera.singleCounter > camera.noneCounter && camera.singleCounter > camera.quadCounter) {
                                rings = 1;
                            } else {
                                rings = 4;
                            }
                        }


                        break;
                    case DriveToShoot:
                        target = (new Pos(60, 11, new Angle(0)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 0.5)) {
                            action = program.Shoot1;
                            timer.reset();
                            shooter.push3.reset();
                        }
                        break;
                    case Shoot1:
                        camera.servoBack();
                        target = (new Pos(60, 11, new Angle(0)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        shooter.push1Ring();
                        if (timer.seconds() > 1) {
                            action = program.ShootDrive1;
                        }

                        break;
                    case ShootDrive1:
                        target = (new Pos(60, 18, new Angle(0)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 0.5)) {
                            action = program.Shoot2;
                            timer.reset();
                            shooter.push3.reset();
                            dt.stop();
                        }
                        break;
                    case Shoot2:
                        target = (new Pos(60, 18, new Angle(0)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        shooter.push1RingWithoutPrep();
                        if (timer.seconds() > .5) {
                            action = program.ShootDrive2;
                        }
                        break;
                    case ShootDrive2:
                        target = (new Pos(60, 24, new Angle(0)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 0.5)) {
                            action = program.Shoot3;
                            timer.reset();
                            shooter.push3.reset();
                            dt.stop();
                        }
                        break;
                    case Shoot3:
                        target = (new Pos(60, 24, new Angle(0)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);

                        shooter.push1RingWithoutPrep();
                        if (timer.seconds() > .5) {
                            action = program.DriveToWobble1;
                            shooter.target = 0;
                        }
                        break;
                    case DriveToWobble1:
                        if (rings == 0) { /// rings
                            target = (new Pos(88, -20, new Angle().setDegrees(-90)));
                        } else if (rings == 1) { // 1 rings
                            target = (new Pos(84, -4, new Angle(0)));
                        } else { // 4 rings
                            target = (new Pos(114, -14, new Angle().setDegrees(-45)));
                        }
                        wobbleArm.setPosition(3200, 1);
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                            action = program.Wobble1Arm;
                            timer.reset();
                            dt.stop();
                        }
                        break;
                    case Wobble1Arm:

                        if (wobbleArm.setPosition(3200, 1)) {
                            wobbleArm.servoOpen();
                            action = program.WaitAfterWobble1;
                            timer.reset();
                        }
                        break;

                    case WaitAfterWobble1:

                        if (timer.seconds() > .1) {
                            action = program.DepositWobble1;
                        }
                        break;

                    case DepositWobble1:
                        if (rings == 0) { /// rings
                            target = (new Pos(72, 0, new Angle().setDegrees(-90)));
                        } else if (rings == 1) { // 1 rings
                            target = (new Pos(72, -12, new Angle().setDegrees(0)));
                        } else { // 4 rings
                            target = (new Pos(72, 0, new Angle().setDegrees(0)));
                        }
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 10, 10)) {
                            action = program.DriveToWobble2;
                            timer.reset();
                            dt.stop();
                        }
                        break;
                    case DriveToWobble2:
                        target = (new Pos(33, -18, new Angle().setDegrees((rings == 0) ? -160 : 200)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                            action = program.CloseGrabber;
                            timer.reset();
                            dt.stop();
                        }
                        break;
                    case CloseGrabber:
                        wobbleArm.servoClose();
                        if (timer.seconds() > 1) {
                            action = program.PrepWobble2;
                        }
                        break;
                    case PrepWobble2:
                        wobbleArm.setPosition(2500, 0.5);
                        target = (new Pos(48, 0, new Angle().setDegrees((rings == 0) ? -90 : 360)));
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 10, 10)) {
                            action = program.DepositWobble2;
                            timer.reset();
                            dt.stop();
                        }
                        break;
                    case DepositWobble2:
                        wobbleArm.setPosition(2500, 0.5);
                        if (rings == 0) { /// rings
                            target = (new Pos(78, -20, new Angle().setDegrees(-90)));
                        } else if (rings == 1) { // 1 rings
                            target = (new Pos(76, -4, new Angle().setDegrees(360)));
                        } else { // 4 rings
                            target = (new Pos(108, -20, new Angle().setDegrees(315)));
                        }
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                            timer.reset();
                            dt.stop();
                            wobbleArm.servoOpen();
                            action = program.WaitAfterWobble2;
                        }
                        break;
                    case WaitAfterWobble2:

                        if (timer.seconds() > 0.1) {
                            action = program.Finish;
                        }

                        break;
                    case Finish:
                        if (rings == 0) { /// rings
                            target = (new Pos(72, 0, new Angle().setDegrees(-90)));
                        } else if (rings == 1) { // 1 rings
                            target = (new Pos(72, -12, new Angle().setDegrees(360)));
                        } else { // 4 rings
                            target = (new Pos(72, 24, new Angle().setDegrees(315)));
                        }
                        dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                        if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                            dt.stop();
                            action = program.Stop;
                        }
                        break;
                }


                shooter.update(loopTime.seconds());
                loopTime.reset();
                telemetry.update();
            }
        }

        savePosition(dt.trackerWheels.pos);
        //dt.stop();

        //camera.stopVuforia();
        //camera.stopTensorFlow();
    }

}
