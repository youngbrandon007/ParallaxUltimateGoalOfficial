package org.firstinspires.ftc.teamcode.teamcode.prometheus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Encoder;

public class TrackerWheels {

    private OpMode opMode;

    private Encoder x1;
    private Encoder x2;
    private Encoder y1;

    public Pos position;

    public TrackerWheels(OpMode opMode){
        this.opMode = opMode;

        position = new Pos();


        //TODO fix wheel radius
        x1 = new Encoder();
        x1.tickPerRotation = 4096;
        x1.wheelRadius = 1;
        x2 = new Encoder();
        x2.tickPerRotation = 4096;
        x2.wheelRadius = 1;
        y1 = new Encoder();
        y1.tickPerRotation = 4096;
        y1.wheelRadius = 1;
    }

    public void reset(int x1tick, int x2tick, int y1tick){
        x1.reset(x1tick);
        x2.reset(x2tick);
        y1.reset(y1tick);
    }

    public void update(int x1tick, int x2tick, int y1tick){
        x1.update(x1tick);
        x2.update(x2tick);
        y1.update(y1tick);
    }

}
