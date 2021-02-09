package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Encoder;

public class TrackerWheels {
    private OpMode opMode;

    private Encoder x1encoder;
    private Encoder x2encoder;
    private Encoder y1encoder;
    double ticksPerRotation = 8192;
    double encWheelDiameter = 2.28346;


    public Pos pos;
    public Pos oldPos;

    public Pos velocity;
    public Pos oldVelocity;


    public TrackerWheels(OpMode opMode){
        this.opMode = opMode;

        pos = new Pos();
        velocity = new Pos();

        x1encoder = new Encoder();
        x1encoder.tickPerRotation = ticksPerRotation;
        x1encoder.wheelDiameter = encWheelDiameter;
        x2encoder = new Encoder();
        x2encoder.tickPerRotation = ticksPerRotation;
        x2encoder.wheelDiameter = encWheelDiameter;
        y1encoder = new Encoder();
        y1encoder.tickPerRotation = ticksPerRotation;
        y1encoder.wheelDiameter = encWheelDiameter;
    }

    @Deprecated
    private Pos calcVelocity(double elapsedTime){
        Pos velocity = new Pos();
        velocity.x = (pos.x - oldPos.x)/elapsedTime;
        velocity.y = (pos.y - oldPos.y)/elapsedTime;
        velocity.angle = new Angle ((pos.angle.getRadians() - oldPos.angle.getRadians())/elapsedTime);
        return velocity;
    }

    public Pos getAcceleration(double elapsedTime){
        Pos acceleration = new Pos();
        acceleration.x = (velocity.x - oldVelocity.x)/elapsedTime;
        acceleration.y = (velocity.y - oldVelocity.y)/elapsedTime;
        acceleration.angle = new Angle ((velocity.angle.getRadians() - oldVelocity.angle.getRadians())/elapsedTime);
        return acceleration;
    }

    public void reset(int x1tick, int x2tick, int y1tick){
        x1encoder.reset(x1tick);
        x2encoder.reset(x2tick);
        y1encoder.reset(y1tick);
    }

    public void update(int encX1, int encX2, int encY, double time){
        oldPos = pos.copy();
        x1encoder.update(encX1);
        x2encoder.update(encX2);
        y1encoder.update(encY);
        double x1;
        double x2;
        double y;
        double distanceX;
        double distanceY;
        double totalCir = 9*Math.PI;
        Angle turn;

        x1 = x1encoder.distance();
        x2 = x2encoder.distance();
        y = y1encoder.distance();

        distanceX = ((x1+x2)/2);
        distanceY = y;
        turn = new Angle().setDegrees((((x1-x2)/totalCir)*360)/2);
        Angle finalAngle = new Angle().setDegrees(pos.angle.getDegrees()+turn.getDegrees());
        Angle change = new Angle().setDegrees((pos.angle.getDegrees()+finalAngle.getDegrees())/2);

        pos.x += (distanceX * Math.cos(change.getRadians())) -  Math.sin(change.getRadians())*distanceY;
        pos.y += (distanceY * Math.cos(change.getRadians())) +  Math.sin(change.getRadians())*distanceX;
        pos.angle = finalAngle;

        oldVelocity = velocity.copy();
        velocity = new Pos();
        velocity.x = distanceX / time;
        velocity.y = distanceY / time;
        velocity.angle.setDegrees(turn.getDegrees() / time);

        opMode.telemetry.addData("Position",pos);

    }
}


