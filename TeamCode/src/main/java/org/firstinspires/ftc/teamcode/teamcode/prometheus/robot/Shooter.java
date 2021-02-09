package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Shooter {
   public DcMotor shooter;

    private ServoImplEx leftIndexer;
    private ServoImplEx rightIndexer;
    public boolean indexerUp;
    private ServoImplEx pusher;
    private ServoImplEx shooterPusher;
    private ServoImplEx shooterLiftRight;

    private boolean shooterUp;

    private OpMode opMode;

    public double f = .000380;
    public double p = .00050;
    public double i = 2.25E-8;
    public double d = .0000;
    public double sumError;
    public double previous;
    public double prevError = 0;
    public double target = 1600;

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
        if(shooterUp){
            leftIndexer.setPosition(0.47);
            rightIndexer.setPosition(0.50);
        }else {
            leftIndexer.setPosition(0.49);
            rightIndexer.setPosition(0.48);
        }
        indexerUp = true;
    }

    public void indexerDown(){
        indexerOn();
        leftIndexer.setPosition(0.61);
        rightIndexer.setPosition(0.36);
        indexerUp = false;
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
        pusher.setPosition(0.45);
    }

    public void pusherBack(){
        pusher.setPosition(0.70);
    }

    public void shooterPusherOut(){
        shooterPusher.setPosition(0.43);
    }

    public void shooterPusherBack(){
        shooterPusher.setPosition(0.31);
    }


    public void shooterLiftUp(){
        shooterUp = true;
        shooterLiftRight.setPosition(.2);
        if(indexerUp){
            indexerUp();
        }
    }

    public void shooterLiftDown(){
        shooterUp = false;
        shooterLiftRight.setPosition(.8);
        if(indexerUp){
            indexerUp();
        }
    }
}