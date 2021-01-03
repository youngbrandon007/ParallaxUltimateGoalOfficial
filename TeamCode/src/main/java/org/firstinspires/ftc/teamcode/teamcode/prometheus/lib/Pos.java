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

}
