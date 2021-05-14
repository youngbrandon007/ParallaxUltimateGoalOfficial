package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;

import android.sax.TextElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    ElapsedTime totalTime = new ElapsedTime();

    MotionProfile moveProfile = new MotionProfile(100, 120);
    MotionProfile rotProfile = new MotionProfile(4, 3);

    MotionProfile slowMoveProfile = new MotionProfile(100, 50);
    MotionProfile slowRotProfile = new MotionProfile(3, 2);

    MotionProfile collectionProfile = new MotionProfile(70, 100);

    enum program {
        DriveForward,
        Camera,
        DriveToShoot,
        Shoot1, ShootDrive1, Shoot2, ShootDrive2, Shoot3,
        DriveToWobble1,
        Wobble1Arm,
        WaitAfterWobble1,
        DepositWobble1,
        DriveToPrepWobble2,
        WobbleArmDown,
        DriveToWobble2,
        CloseGrabber,
        PrepWobble2,
        DepositWobble2,
        WaitAfterWobble2,
        Finish,
        Stop,

        DriveToGoal,
        GoalShoot,
        Collection1,
        FinishCollection1,

        DriveToGoal4,
        GoalShoot4,
        Collection4,
        FinishCollection4,
        DriveToGoal24,
        GoalShoot24,
        Collection24,
        FinishCollection24,
    }

    program action = program.DriveForward;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
            telemetry.setAutoClear(false);


            dt = new DriveTrain(this);
            shooter = new Shooter(this);
            collector = new Collector(this);
            wobbleArm = new WobbleArm(this, false);
            camera = new Camera(this);

            camera.initCamera();
            camera.servoBack();
            dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            shooter.shooterPusherOut();
            shooter.pusherBack();
            shooter.shooterLiftMiddle();
            shooter.indexerDown();

            wobbleArm.servoOpen();
            wobbleArm.wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            timer.reset();

            while(!isStarted() && !isStopRequested()){
                if(wobbleArm.mag.isPressed()) {
                    wobbleArm.wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    wobbleArm.wobbleArm.setPower(-0.1);
                }else{

                    wobbleArm.wobbleArm.setPower(0.0);
                }

                if(wobbleArm.button.isPressed()){
                    wobbleArm.servoOpen();
                }else if(wobbleArm.wobbleSensed()) {
                    wobbleArm.servoClose();
                }


                if(timer.seconds() > 1.5) {
                    shooter.indexerAutoInit();
                }else  if(timer.seconds() > 1) {
                    shooter.pusherBack();
                } else if(timer.seconds() > 0.5) {
                    shooter.pusherOut();
                }

                telemetry.addData("Distance", wobbleArm.distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
            }


            waitForStart();

            totalTime.reset();

            wobbleArm.servoClose();

            shooter.shooterLiftMiddle();

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


            wobbleArm.wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive() && totalTime.seconds() < 29.5) {
                if (loopTime.milliseconds() > 50) {
                    double time = loopTime.seconds();
                    telemetry.addData("Time", time);
                    telemetry.addData("Shooter Target", shooter.target);
                    shooter.update(time);
                    loopTime.reset();

                    //camera.vuforiaLoop();
                    camera.tensorFlowLoop();


                    dt.updateTrackerWheels(time);
                    switch (action) {

                        case DriveForward:

                            shooter.indexerUp();
                            wobbleArm.setPosition(500, 0.6);
                            wobbleArm.servoClose();

                            shooter.target = shooter.ShooterGoalSpeed;


                            target = (new Pos(18, -8, new Angle().setDegrees(-20)));
                            dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 2, 2)) {
                                timer.reset();
                                camera.noneCounter = 0;
                                camera.singleCounter = 0;
                                camera.quadCounter = 0;

                                action = program.Camera;
                                dt.stop();
                            }
                            break;
                        case Camera:

                            shooter.indexerUp();
                            wobbleArm.setPosition(500, 0.6);
                            shooter.indexerUp();
                            shooter.shooterPusherBack();
                            if (timer.seconds() > 2) {
                                camera.stopTensorFlow();

                                action = program.DriveToShoot;
                                dt.resetPID();
                                if (camera.noneCounter > camera.singleCounter && camera.noneCounter > camera.quadCounter) {
                                    rings = 0;
                                } else if (camera.singleCounter > camera.noneCounter && camera.singleCounter > camera.quadCounter) {
                                    rings = 1;
                                } else {
                                    rings = 4;
                                }

                                if(rings == 1){
                                    action = program.DriveToGoal;
                                    shooter.target = shooter.ShooterAutoFirstSpeed;
                                    timer.reset();
                                }
                                if(rings == 4){
                                    action = program.DriveToGoal4;
                                    shooter.target = shooter.ShooterAutoFirstSpeed;
                                    timer.reset();
                                }
                            }


                            break;
                        case DriveToGoal:
                            shooter.shooterLiftMiddle();
                            target = (new Pos(22, -8, new Angle().setDegrees(-1)));
                            dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                                timer.reset();
                                shooter.push3.reset();
                                shooter.indexerUp();
                                action = program.GoalShoot;
                                dt.stop();

                                shooter.pusherBack();
                            }
                            break;
                        case GoalShoot:
                            shooter.push1RingWithoutPrep();
                            if(shooter.push3.seconds() > .5){
                                timer.reset();

                                shooter.target = shooter.ShooterGoalSpeed;
                                action = program.Collection1;


                                shooter.indexerDown();
                                shooter.shooterLiftUp();
                                collector.collector.setPower(1.0);
                            }
                            break;
                        case Collection1:
                            target = (new Pos(45, -8, new Angle().setDegrees(0)));
                            if(timer.seconds() > 0.5) {
                                dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            }

                            if(timer.seconds() > 0.5) {
                                if ((int) (timer.seconds() * 5) % 2 == 0) {
                                    shooter.pusherBack();
                                } else {
                                    shooter.pusherOut();
                                }
                            }

                            if (timer.seconds() > 3) {

                                shooter.pusherOut();

                                if(timer.seconds() > 3.3){
                                    timer.reset();
                                    action = program.FinishCollection1;
                                    dt.stop();
                                    shooter.push3.reset();

                                    collector.collector.setPower(0);
                                    shooter.shooterLiftMiddle();
                                }
                            }



                            break;
                        case FinishCollection1:
                            if(shooter.indexerUpSequence()){
                                timer.reset();
                                dt.resetPID();
                                action = program.DriveToShoot;
                            }
                            break;
                        case DriveToGoal4:
                            shooter.shooterLiftMiddle();
                            target = (new Pos(22, -8, new Angle().setDegrees(-1)));
                            dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                                timer.reset();
                                shooter.push3.reset();
                                shooter.indexerUp();
                                action = program.GoalShoot4;
                                dt.stop();

                                shooter.pusherBack();
                            }
                            break;
                        case GoalShoot4:
                            shooter.push3RingsWithoutPrep();
                            if(shooter.push3.seconds() > 1.1){
                                timer.reset();

                                shooter.target = shooter.ShooterAutoFirstSpeed;
                                action = program.Collection4;


                                shooter.indexerDown();
                                shooter.shooterLiftUp();
                                collector.collector.setPower(-1.0);
                            }
                            break;
                        case Collection4:
                            target = (new Pos(34, -8, new Angle().setDegrees(0)));
                            if(timer.seconds() > 0.8){
                                target = (new Pos(28, -8, new Angle().setDegrees(0)));
                            }



//                            if(timer.seconds() > 1.3){
//                                target = (new Pos(34, -8, new Angle().setDegrees(0)));
//                            }

//                            if(timer.seconds() > 2.0){
//                                target = (new Pos(28, -8, new Angle().setDegrees(0)));
//                            }

                            if(timer.seconds() > 1.2){
                                collector.collector.setPower(1.0);
                            }
                            if(timer.seconds() > 0.0) {
                                if(timer.seconds() > 1.3 && timer.seconds() < 2.0){
                                    dt.setFromAxis(-0.3, 0, 0);
                                }else {
                                    dt.updateMovement(target, collectionProfile, rotProfile, time, true);
                                }
                            }

                            if(timer.seconds() > 0.5) {
                                if ((int) (timer.seconds() * 5) % 2 == 0) {
                                    shooter.pusherBack();
                                } else {
                                    shooter.pusherOut();
                                }
                            }

                            if (timer.seconds() > 3) {

                                shooter.pusherOut();

                                if(timer.seconds() > 3.3){
                                    timer.reset();
                                    action = program.FinishCollection4;
                                    dt.stop();
                                    shooter.push3.reset();

                                    collector.collector.setPower(0);
                                    shooter.shooterLiftMiddle();
                                }
                            }



                            break;
                        case FinishCollection4:

                            //TEMP TESTING
                            shooter.shooterLiftMiddle();
                            target = (new Pos(30, -8, new Angle().setDegrees(-1)));
                            dt.updateMovement(target, slowMoveProfile, slowRotProfile, time, true);

                            if(shooter.indexerUpSequence()){
                                timer.reset();
                                dt.resetPID();
                                //SKIPPING NEXT ONE TEMP
                                action = program.DriveToShoot;

                                timer.reset();
                                shooter.push3.reset();
                                shooter.indexerUp();
                                dt.stop();

                                shooter.pusherBack();
                            }
                            break;
                        case DriveToGoal24:
                            shooter.shooterLiftMiddle();
                            target = (new Pos(26, -8, new Angle().setDegrees(-1)));
                            dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                                timer.reset();
                                shooter.push3.reset();
                                shooter.indexerUp();
                                action = program.GoalShoot24;
                                dt.stop();

                                shooter.pusherBack();
                            }
                            break;
                        case GoalShoot24:
                            shooter.push1RingWithoutPrep();
                            if(shooter.push3.seconds() > 0.5){
                                timer.reset();

                                shooter.target = shooter.ShooterGoalSpeed;
                                action = program.Collection24;


                                shooter.indexerDown();
                                shooter.shooterLiftUp();
                                collector.collector.setPower(1.0);
                            }
                            break;
                        case Collection24:
                            target = (new Pos(45, -8, new Angle().setDegrees(0)));
                            if(timer.seconds() > 0.5) {
                                dt.setFromAxis(-0.3, 0, 0);
                                //dt.updateMovement(target, collectionProfile, rotProfile, time, true);
                            }


                            if(timer.seconds() > 0.5) {
                                if ((int) (timer.seconds() * 5) % 2 == 0) {
                                    shooter.pusherBack();
                                } else {
                                    shooter.pusherOut();
                                }
                            }

                            if (timer.seconds() > 5) {

                                shooter.pusherOut();

                                if(timer.seconds() > 5.3){
                                    timer.reset();
                                    action = program.FinishCollection24;
                                    dt.stop();
                                    shooter.push3.reset();

                                    collector.collector.setPower(0);
                                    shooter.shooterLiftMiddle();
                                }
                            }



                            break;
                        case FinishCollection24:

                            if(shooter.indexerUpSequence()){
                                timer.reset();
                                dt.resetPID();
                                action = program.DriveToShoot;
                            }
                            break;
                        case DriveToShoot:
                            target = (new Pos(52, -6, new Angle(Math.toRadians(0))));
                            dt.updateMovement(target, moveProfile, slowRotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 1, 0.5)) {
                                action = program.Shoot1;
                                timer.reset();
                                shooter.push3.reset();
                                dt.resetPID();
                            }
                            break;
                        case Shoot1:
                            camera.servoBack();
                            target = (new Pos(52, -6, new Angle(Math.toRadians(0))));
                            dt.updateMovement(target, slowMoveProfile, slowRotProfile, time, true);
                            shooter.push1Ring();
                            if (timer.seconds() > 1.0) {
                                action = program.ShootDrive1;
                                dt.resetPID();
                            }

                            break;
                        case ShootDrive1:
                            target = (new Pos(52, -6, new Angle(0)));
                            dt.updateMovement(target, moveProfile, slowRotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 2, 1)) {
                                action = program.Shoot2;
                                timer.reset();
                                shooter.push3.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case Shoot2:
                            target = (new Pos(52, -6, new Angle(0)));
                            dt.updateMovement(target, slowMoveProfile, slowRotProfile, time, true);
                            shooter.push1RingWithoutPrep();
                            if (timer.seconds() > 0.5) {
                                action = program.ShootDrive2;
                                dt.resetPID();
                            }
                            break;
                        case ShootDrive2:
                            target = (new Pos(52, -6, new Angle(Math.toRadians(0))));
                            dt.updateMovement(target, moveProfile, slowRotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 2, 1)) {
                                action = program.Shoot3;
                                timer.reset();
                                shooter.push3.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case Shoot3:
                            target = (new Pos(52, -6, new Angle(Math.toRadians(0))));
                            dt.updateMovement(target, slowMoveProfile, slowRotProfile, time, true);

                            shooter.push1RingWithoutPrep();
                            if (timer.seconds() > 0.5) {
                                action = program.DriveToWobble1;
                                shooter.target = 0;
                                dt.resetPID();
                            }
                            break;
                        case DriveToWobble1:
                            if (rings == 0) { /// rings
                                target = (new Pos(86, -20, new Angle().setDegrees(-90)));
                            } else if (rings == 1) { // 1 rings
                                target = (new Pos(84, -2, new Angle(0)));
                            } else { // 4 rings
                                target = (new Pos(114, -14, new Angle().setDegrees(-45)));
                            }
                            wobbleArm.setPosition(2900, 1);
                            dt.updateMovement(target, slowMoveProfile, rotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 2, 2)) {
                                action = program.Wobble1Arm;
                                timer.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case Wobble1Arm:

                            if (wobbleArm.setPosition(2900, 1)) {
                                wobbleArm.servoOpen();
                                action = program.WaitAfterWobble1;
                                timer.reset();
                            }
                            break;

                        case WaitAfterWobble1:

                            if (timer.seconds() > .3) {
                                action = program.DepositWobble1;
                                dt.resetPID();
                            }
                            break;

                        case DepositWobble1:
                            if (rings == 0) { /// rings
                                target = (new Pos(40, -15, new Angle().setDegrees(-180)));
                            } else if (rings == 1) { // 1 rings
                                target = (new Pos(72, -10, new Angle().setDegrees(0)));
                            } else { // 4 rings
                                target = (new Pos(72, 0, new Angle().setDegrees(0)));
                            }
                            dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            wobbleArm.setPosition(2700, 1);
                            if (target.atPos(dt.trackerWheels.pos, 15, 15)) {
                                action = program.DriveToPrepWobble2;
                                timer.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case DriveToPrepWobble2:
                            target = (new Pos(32, -22, new Angle().setDegrees((rings == 0) ? -180 : 180)));
                            dt.updateMovement(target, moveProfile, slowRotProfile, time, true);
                            wobbleArm.setPosition(2700, 1);
                            if (target.atPos(dt.trackerWheels.pos, 1, 1)) {
                                action = program.WobbleArmDown;
                                timer.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case WobbleArmDown:
                            if(wobbleArm.setPosition(3100, 1)){
                                action = program.DriveToWobble2;
                                timer.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case DriveToWobble2:
                            target = (new Pos(32, -26, new Angle().setDegrees((rings == 0) ? -140 : 220)));
                            dt.updateMovement(target, slowMoveProfile, new MotionProfile(1.5, 3), time, true);
                            if (target.atPos(dt.trackerWheels.pos, 1, 1) || wobbleArm.wobbleSensed()) {
                                action = program.CloseGrabber;
                                timer.reset();
                                dt.stop();
                                dt.resetPID();
                            }


                            break;
                        case CloseGrabber:
                            wobbleArm.servoClose();
                            if (timer.seconds() > 1) {
                                action = program.PrepWobble2;
                                dt.resetPID();
                            }
                            break;
                        case PrepWobble2:
                            wobbleArm.setPosition(2500, 0.5);
                            target = (new Pos(48, 0, new Angle().setDegrees((rings == 0) ? -90 : 360)));
                            dt.updateMovement(target, slowMoveProfile, slowRotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 10, 20)) {
                                action = program.DepositWobble2;
                                timer.reset();
                                dt.stop();
                                dt.resetPID();
                            }
                            break;
                        case DepositWobble2:
                            wobbleArm.setPosition(2500, 0.5);
                            if (rings == 0) { /// rings
                                target = (new Pos(78, -20, new Angle().setDegrees(-90)));
                            } else if (rings == 1) { // 1 rings
                                target = (new Pos(82, -12, new Angle().setDegrees(360)));
                            } else { // 4 rings
                                target = (new Pos(112, -20, new Angle().setDegrees(315)));
                            }
                            dt.updateMovement(target, slowMoveProfile, slowRotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 2, 2)) {
                                timer.reset();
                                dt.stop();
                                action = program.WaitAfterWobble2;
                                dt.resetPID();
                            }
                            break;
                        case WaitAfterWobble2:

                            if (timer.seconds() > 0.4) {

                                wobbleArm.servoOpen();
                                if(timer.seconds() > 0.8){
                                    action = program.Finish;
                                    dt.resetPID();
                                }
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
                            dt.updateMovement(target, moveProfile, rotProfile, time, true);
                            if (target.atPos(dt.trackerWheels.pos, 2, 2)) {
                                dt.stop();
                                action = program.Stop;
                                dt.resetPID();
                            }
                            break;
                        case Stop:
                            wobbleArm.setPosition(0, 1.0);
                            shooter.indexerDown();
                            break;
                    }

                    telemetry.update();
                }
            }


            dt.stop();
            wobbleArm.wobbleArm.setPower(0);
            wobbleArm.servoOpen();
            savePosition(dt.trackerWheels.pos);


            //camera.stopVuforia();
            //camera.stopTensorFlow();
        }catch (Exception e){
            telemetry.addData("Error", e);
        }
    }
}
