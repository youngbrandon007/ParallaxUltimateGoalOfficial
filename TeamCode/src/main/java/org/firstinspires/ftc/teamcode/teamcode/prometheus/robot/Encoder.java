package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;

public class Encoder {

    private int curr;
    private int prev;

    public double tickPerRotation;
    public double wheelRadius;

    public void reset(int newValue){
        curr = newValue;
        prev = newValue;
    }

    public void update(int newValue){
        prev = curr;
        curr = newValue;
    }

    public int getTickChange(){
        return curr - prev;
    }

    public double getRotationChangePercent(){
        return (double) getTickChange() / tickPerRotation;
    }

    public Angle getAngleChange(){
        return new Angle(getRotationChangePercent() * 2 * Math.PI);
    }

}
