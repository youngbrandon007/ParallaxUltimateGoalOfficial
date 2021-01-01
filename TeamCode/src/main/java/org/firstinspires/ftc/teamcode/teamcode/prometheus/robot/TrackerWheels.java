package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Encoder;

public class TrackerWheels {

    private OpMode opMode;
//
//    private Encoder x1;
//    private Encoder x2;
//    private Encoder y1;
double oldX1;
    double oldX2;
    double oldY;

    public Pos pos;

    public TrackerWheels(OpMode opMode){
        this.opMode = opMode;

        pos = new Pos();


//        //TODO fix wheel radius
//        x1 = new Encoder();
//        x1.tickPerRotation = 4096;
//        x1.wheelRadius = 1;
//        x2 = new Encoder();
//        x2.tickPerRotation = 4096;
//        x2.wheelRadius = 1;
//        y1 = new Encoder();
//        y1.tickPerRotation = 4096;
//        y1.wheelRadius = 1;
    }

//    public void reset(int x1tick, int x2tick, int y1tick){
//        x1.reset(x1tick);
//        x2.reset(x2tick);
//        y1.reset(y1tick);
//    }

    public void update(int encX1, int encX2, int encY){
//        x1.update(x1tick);
//        x2.update(x2tick);
//        y1.update(y1tick);
        double x1;
        double x2;
        double y;
        double distanceX;
        double distanceY;
        double turnDistance;

        double totalCir = 9*Math.PI;
        Angle turn;



                x1 = (double) (encX1-oldX1)/8192 * 2.28346 * Math.PI;
                x2 = (double) (encX2-oldX2)/8192 * 2.28346 * Math.PI;
                y = (double) (encY-oldY)/8192 * 2.28346 * Math.PI;




                distanceX = ((x1+x2)/2);
                distanceY = y;
                turn = new Angle().setDegrees((((x1-x2)/totalCir)*360)/2);
                Angle finalAngle = new Angle().setDegrees(pos.angle.getDegrees()+turn.getDegrees());
                Angle change = new Angle().setDegrees((pos.angle.getDegrees()+finalAngle.getDegrees())/2);

                pos.x += (distanceX * Math.cos(change.getRadians())) -  Math.sin(change.getRadians())*distanceY;
                pos.y += (distanceY * Math.cos(change.getRadians())) +  Math.sin(change.getRadians())*distanceX;
                pos.angle = finalAngle;

                opMode.telemetry.addData("distanceX", distanceX);
                opMode.telemetry.addData("distanceY", distanceY);
                opMode.telemetry.addData("angle", turn.deg());
                opMode.telemetry.addData("positionX",pos.x);
                opMode.telemetry.addData("positionY",pos.y);
                opMode.telemetry.addData("positionAngle",pos.angle.getDegrees());




                oldX1 = encX1;
                oldX2 = encX2;
                oldY = encY;























            }
        }


