package org.firstinspires.ftc.teamcode.teamcode.prometheus.lib;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;

public class Pos {

    public double x;
    public double y;

    public Angle angle;

    public Pos(){
        x = 0;
        y = 0;
        angle = new Angle();
    }

    public Pos(double x, double y, Angle angle){
        this.x = x;
        this.y = y;

        this.angle = new Angle(angle);
    }

    public Pos copy(){

        return new Pos(x,y,angle.copy());
    }

    public Pos sub(Pos other){
        return new Pos(x - other.x, y - other.y, new Angle(angle.getRadians() - other.angle.getRadians()));
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public Pos rotate(Angle r){
        double sin = Math.sin(r.getRadians());
        double cos = Math.cos(r.getRadians());
        return new Pos(x * cos - y * sin, y * cos + x * sin, new Angle(angle.getRadians() + r.getRadians()));
    }

    public Angle translationAngle(){
        return new Angle(Math.atan2(y, x));
    }

    public String toString(){
        return "(" + x + ", " + y + ") - " + angle.deg();
    }
}
