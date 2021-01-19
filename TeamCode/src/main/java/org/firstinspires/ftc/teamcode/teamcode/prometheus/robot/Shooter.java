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

    private OpMode opMode;

    public double p = .000380;
    public double i = 1.5E-8;
    public double d = .00100;
    public double sumError;
    public double previous;
    public double target = 1650;

    public Shooter(OpMode opMode) {
        this.opMode = opMode;
        shooter = opMode.hardwareMap.get(DcMotor.class, "s");
        previous = -shooter.getCurrentPosition();

        leftIndexer = opMode.hardwareMap.get(ServoImplEx.class, "il");
        rightIndexer = opMode.hardwareMap.get(ServoImplEx.class, "ir");
        pusher = opMode.hardwareMap.get(ServoImplEx.class, "p");

        shooterPusher = opMode.hardwareMap.get(ServoImplEx.class, "sp");
    }

    public void update(double time) {
        double current = -shooter.getCurrentPosition();
        double speed = (current - previous)/time;
        double error = target - speed;
        sumError += error;
        double pid = (p * target) + (i * sumError * target) + (d * error);
        shooter.setPower(-pid);
        opMode.telemetry.addData("PID Value", pid);
        opMode.telemetry.addData("Ticks", speed);
        opMode.telemetry.addData("rpm", speed/28 * 60);

        previous = current;
    }

    public void indexerUp(){
        leftIndexer.setPosition(0.46);
        rightIndexer.setPosition(0.51);
        indexerUp = true;
    }

    public void indexerDown(){
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
        shooterPusher.setPosition(0.55);
    }

    public void shooterPusherBack(){
        shooterPusher.setPosition(0.31);
    }
}