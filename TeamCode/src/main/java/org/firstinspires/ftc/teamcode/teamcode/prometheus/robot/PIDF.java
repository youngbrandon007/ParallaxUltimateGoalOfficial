package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

public class PIDF {

    public double p;
    public double i;
    public double d;
    public double f;

    private double errorSum = 0;

    private double prevError = 0;

    public PIDF(double p, double i , double d, double f){
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public void resetI(){
        errorSum = 0;
    }

    public double update(double val, double target, double time){
        double error = target - val;

        double derv = (prevError - error) / time;
        errorSum += error;

        prevError = error;

        return p * error + i * errorSum + d * derv + f * target;
    }

    public String toString(){
        return "P: " + p + "\nI: " + i + "\nD: " + d + "\nF: " + f;
    }
}
