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
    public double p = 5.100000000000006E-4;
    public double i = 5.500000000000002E-9; // 9.5000000000000016E-9
    public double d = .0000;
    public double sumError;
    public double previous;
    public double prevError = 0;
    public final double ShooterGoalSpeed = 2400;
    public final double ShooterPowerSpeed = 2100;
    public final double ShooterPowerSpeedTele = 2000;
    public double target = 0;

    public ElapsedTime push3 = new ElapsedTime();
    public boolean autoShoot = false;

    public ElapsedTime pusherTimer = new ElapsedTime();
    public boolean pusherTimerOn = false;

    public enum ShooterState{
        Collecting, ActionPrep, Prep, ActionPrepShooter, PrepShoot,  ActionShoot
    }
    public ShooterState state = ShooterState.Collecting;

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
        if(time <= 0){
            time = 0.1;
            opMode.telemetry.addLine("TIME ERROR");
        }
        double current = -shooter.getCurrentPosition();
        double speed = (current - previous)/time;
        double error = target - speed;
        if(error > 500) {
            shooter.setPower(-1.0);

            opMode.telemetry.addData("PID Value", -1.0 * 1000);
        }else if(error < -500){
            shooter.setPower(1.0);

            opMode.telemetry.addData("PID Value", 1.0 * 1000);
        }else {
            double derv = (error - prevError) / time;
            sumError += error;
            double pid = (f * target) + (p * error)  + (i * sumError * target) + (d * derv);

            if(Math.abs(pid) > 1){
                sumError -= error;
            }
            shooter.setPower(-pid);

            opMode.telemetry.addData("PID Value", pid * 1000);
        }

        opMode.telemetry.addData("Ticks", speed);
        opMode.telemetry.addData("rpm", speed/28 * 60);
        opMode.telemetry.addData("Total I Value", sumError);

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

    public boolean push3RingsWithoutPrep(){
        autoShoot = true;

        if(push3.seconds() < .2){
            shooterPusherOut();
        }
        else if(push3.seconds() < .4){
            shooterPusherBack();
        }
        else if(push3.seconds() < .6){
            shooterPusherOut();
        }
        else if(push3.seconds() < .8){
            shooterPusherBack();
        }
        else if(push3.seconds() < 1.0){
            shooterPusherOut();
        }
        else{
            shooterPusherBack();
            autoShoot = false;
            return true;
        }
        return false;
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

    public boolean indexerUpSequence(){
        if(push3.seconds() < .2){
            pusherBack();
        }
        else if(push3.seconds() < .7){
            indexerUp();
        }
        else {
            return true;
        }
        return false;
    }
}