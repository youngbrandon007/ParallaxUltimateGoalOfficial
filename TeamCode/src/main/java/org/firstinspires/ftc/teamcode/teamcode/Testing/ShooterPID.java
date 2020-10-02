package org.firstinspires.ftc.teamcode.teamcode.Testing;

public class ShooterPID {

    public double f;
    public double p;
    public double i;
    public double d;


    private double integral = 0;
    private double prevError = 0;

    private double target;

    private double output;

    public ShooterPID(double f, double p, double i, double d){
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double update(double current, double time){
        double error = target - current;

        integral += error * time;

        double derivative = (error - prevError) / time;

        output = target * f + error * p + integral * i + d * derivative;

        return output;
    }

    public void resetI(){
        integral = 0;
    }


    public void setTarget(double target){
        this.target = target;
    }

    public double getTarget(){
        return target;
    }

    public double getOutput(){
        return output;
    }
}
