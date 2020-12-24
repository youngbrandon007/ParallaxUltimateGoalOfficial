package org.firstinspires.ftc.teamcode.teamcode.prometheus.lib;

public class Angle {

    private double radians;

    public Angle(){
        radians = 0;
    }

    public Angle(Angle angle){
        radians = angle.rad();
    }

    public Angle(double radians){
        this.radians = radians;
    }

    public Angle setRadians(double radians){
        this.radians = radians;
        return this;
    }

    public Angle setDegrees(double degrees){
        this.radians = Math.toRadians(degrees);
        return this;
    }

    public double getRadians(){
        return radians;
    }

    public double getDegrees(){
        return Math.toDegrees(radians);
    }

    //Shorter versions

    public Angle rad(double radians){
        return setRadians(radians);
    }

    public double rad(){
        return getRadians();
    }

    public Angle deg(double degrees){
        return setDegrees(degrees);
    }

    public double deg(){
        return getDegrees();
    }
}
