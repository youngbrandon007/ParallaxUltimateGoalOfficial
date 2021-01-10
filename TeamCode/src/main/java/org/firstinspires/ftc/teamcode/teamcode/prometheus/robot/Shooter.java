package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter {

   public DcMotor shooter;

    private OpMode opMode;

    public double p = .000331;
    public double i = 0;
    public double d;
    public double sumError;
    public double previous;
    public double target = 0;

    public Shooter(OpMode opMode) {
        this.opMode = opMode;
        shooter = opMode.hardwareMap.get(DcMotor.class, "s");
        previous = shooter.getCurrentPosition();
    }

    public void update(double time) {
        double current = shooter.getCurrentPosition();
        double speed = (current - previous)/time;
        double error = target - speed;
        sumError += error;
        double pid = (p * target) + (i * sumError);
        shooter.setPower(pid);
        opMode.telemetry.addData("PID Value", pid);
        opMode.telemetry.addData("Speed", speed);
        opMode.telemetry.addData("rpm", speed/28);

        previous = current;
    }
}