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
        Shoot,
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

    camera.startVuforia();
    camera.startTensorFlow();
    camera.servoDown();
<<<<<<< HEAD

    loopTime.reset();
    dt.resetTrackerWheels();
=======
    Pos target;
>>>>>>> d11930752ced3c9fb180dcccef6ef9673f681964

    while (opModeIsActive()) {
        if (loopTime.milliseconds() > 50) {
            camera.vuforiaLoop();
            camera.tensorFlowLoop();

            dt.updateTrackerWheels(loopTime.seconds());
            switch (action) {

                case DriveForward:
                    target = (new Pos(24, -12, new Angle(0)));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        action = program.Camera;
                        dt.stop();
                    }
                    break;
                case Camera:
                    if (timer.seconds()>2) {
                        shooter.shooter.setPower(1);
                        action = program.DriveToShoot;
                    }


                    break;
                case DriveToShoot:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        action = program.Shoot;
                        timer.reset();
                        shooter.push3.reset();
                        dt.stop();
                    }
                    break;
                case Shoot:
                    shooter.push3Rings();
                    if (timer.seconds()>2) {
                        action = program.DriveToWobble1;
                        shooter.shooter.setPower(0);
                    }

                    break;
                case DriveToWobble1:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        action = program.DepositWobble1;
                        timer.reset();
                        dt.stop();
                    }
                    break;
                case DepositWobble1:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        action = program.DriveToWobble2;
                        timer.reset();
                        dt.stop();
                    }
                    break;
                case DriveToWobble2:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        action = program.DepositWobble2;
                        timer.reset();
                        dt.stop();
                    }
                    break;
                case DepositWobble2:
                    target = (new Pos(0, 0, new Angle()));
                    dt.updateMovement(target, moveProfile, rotProfile, loopTime.seconds(), true);
                    if (dt.trackerWheels.pos.sub(target).getDistance()<1){
                        timer.reset();
                        dt.stop();
                    }
                    break;
            }


            loopTime.reset();
        }

        camera.stopVuforia();
        camera.stopTensorFlow();

    }
    }

}
