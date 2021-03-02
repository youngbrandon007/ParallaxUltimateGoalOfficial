package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
   public DcMotor shooter;

    private ServoImplEx leftIndexer;
    private ServoImplEx rightIndexer;
    public boolean indexerUp;
    private ServoImplEx pusher;
    private ServoImplEx shooterPusher;
    private ServoImplEx shooterLiftRight;

    private int shooterUp = 0;

    private OpMode opMode;

    public double f = .000380;
    public double p = .00050;
    public double i = 2.25E-8;
    public double d = .0000;
    public double sumError;
    public double previous;
    public double prevError = 0;
    public double target = 1600;

    public ElapsedTime push3 = new ElapsedTime();
    public boolean autoShoot = false;

    public ElapsedTime pusherTimer = new ElapsedTime();
    public boolean pusherTimerOn = false;

    public Shooter(OpMode opMode) {
        this.opMode = opMode;
        shooter = opMode.hardwareMap.get(DcMotor.class, "s");
        previous = -shooter.getCurrentPosition();

        leftIndexer = opMode.hardwareMap.get(ServoImplEx.class, "il");
        rightIndexer = opMode.hardwareMap.get(ServoImplEx.class, "ir");
        pusher = opMode.hardwareMap.get(ServoImplEx.class, "p");

        shooterPusher = opMode.hardwareMap.get(ServoImplEx.class, "sp");

        shooterLiftRight = opMode.hardwareMap.get(ServoImplEx.class, "slr");
    }

    public void update(double time) {
        double current = -shooter.getCurrentPosition();
        double speed = (current - previous)/time;
        double error = target - speed;
        double derv = (error - prevError) / time;
        sumError += error;
        double pid = (f * target) + (p * error)  + (i * sumError * target) + (d * derv);
        shooter.setPower(-pid);
        opMode.telemetry.addData("PID Value", pid);
        opMode.telemetry.addData("Ticks", speed);
        opMode.telemetry.addData("rpm", speed/28 * 60);

        previous = current;
        prevError = error;
    }

    public void indexerUp(){
        indexerOn();
        if(shooterUp == 2) {
            leftIndexer.setPosition(0.45);
            rightIndexer.setPosition(0.52);
        }else if(shooterUp == 1){
            leftIndexer.setPosition(0.46);
            rightIndexer.setPosition(0.51);
        }else {
            leftIndexer.setPosition(0.47);
            rightIndexer.setPosition(0.50);
        }
        indexerUp = true;
    }

    public void indexerDown(){
        indexerOn();
        leftIndexer.setPosition(0.59);
        rightIndexer.setPosition(0.38);
        indexerUp = false;
    }

    public void indexerAutoInit(){
        leftIndexer.setPosition(0.55);
        rightIndexer.setPosition(0.43);
    }

    public void indexerOn(){
        rightIndexer.setPwmEnable();
        leftIndexer.setPwmEnable();
    }

    public void indexerOff(){
        rightIndexer.setPwmDisable();
        leftIndexer.setPwmDisable();
    }

    public void pusherOut(){
        pusher.setPosition(0.61);
    }

    public void pusherBack(){
        pusher.setPosition(0.70);
    }

    public void pusherMiddle() {pusher.setPosition(0.67);}

    public void pusherWait(){
        if(!pusherTimerOn){
            pusherTimer.reset();
        }
        pusherTimerOn = true;
        if(pusherTimer.seconds() > .5 && pusherTimerOn){
            pusherMiddle();
            pusherTimerOn = false;
        }

    }

    public void shooterPusherOut(){
        shooterPusher.setPosition(0.42);
    }

    public void shooterPusherBack(){
        shooterPusher.setPosition(0.31);
    }


    public void shooterLiftUp(){
        shooterUp = 2;
        shooterLiftRight.setPosition(.2);
        if(indexerUp){
            indexerUp();
        }
    }

    public void shooterLiftDown(){
        shooterUp = 0;
        shooterLiftRight.setPosition(.8);
        if(indexerUp){
            indexerUp();
        }
    }

    public void shooterLiftMiddle(){
        shooterUp = 1;
        shooterLiftRight.setPosition(.6);
        if(indexerUp){
            indexerUp();
        }
    }

    public void push3Rings(){
        autoShoot = true;

        if(push3.seconds() < .3){
            pusherBack();
        }else if(push3.seconds() < .7) {
            indexerUp();
            shooterPusherBack();
        }
        else if(push3.seconds() < .9){
            shooterPusherOut();
        }
        else if(push3.seconds() < 1.1){
            shooterPusherBack();
        }
        else if(push3.seconds() < 1.3){
            shooterPusherOut();
        }
        else if(push3.seconds() < 1.5){
            shooterPusherBack();
        }
        else if(push3.seconds() < 1.7){
            shooterPusherOut();
        }
        else if(push3.seconds() < 1.9){
            shooterPusherBack();
        }
        else {
            autoShoot = false;
        }
    }

    public void push1Ring(){
        autoShoot = true;
        if(push3.seconds() < .5) {
            indexerUp();
        }
        else if(push3.seconds() < .7){
            shooterPusherOut();
        }
        else if(push3.seconds() < .9){
            shooterPusherBack();
        }
        else {
            autoShoot = false;
        }
    }

    public void push1RingWithoutPrep(){
        autoShoot = true;
        if(push3.seconds() < .2){
            shooterPusherOut();
        }
        else if(push3.seconds() < .4){
            shooterPusherBack();
        }
        else {
            autoShoot = false;
        }
    }
}