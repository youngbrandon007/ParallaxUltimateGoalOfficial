package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.MotionProfile;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;

public class DriveTrain {

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    public DcMotor[] motors;

    private OpMode opMode;

    public PIDF xPID = new PIDF(0, -0.0011, 0, -0.013);
    public PIDF yPID = new PIDF(0, -0.0011, 0, -0.014);
    public PIDF rPID = new PIDF(0, 0, 0, 0);

    double sumError = 0;

    public TrackerWheels trackerWheels;

    public DriveTrain(OpMode opMode){
        this.opMode = opMode;
        frontRight = opMode.hardwareMap.get(DcMotor.class, "fr");
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "fl");
        backRight = opMode.hardwareMap.get(DcMotor.class, "br");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "bl");

        motors = new DcMotor[]{frontRight, frontLeft, backLeft, backRight};

        trackerWheels = new TrackerWheels(opMode);
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        for(DcMotor m : motors){
            m.setZeroPowerBehavior(behavior);
        }
    }

    // Y is forward/back (forward is positive)
    // X is right/left (left is positive)
    public void setFromAxis(double y, double x, double r, double scale){
        setFromAxis(y * scale, x * scale, r * scale);
    }

    public void setFromAxis(double y, double x, double r){
        double[] values = new double[]{y + x + r, - y + x + r, - y - x + r, y - x + r};

        double max = 1.0;

        for(double num : values){
            if(Math.abs(num) > max){
                max = Math.abs(num);
            }
        }

        for(int i = 0; i < 4; i++) {
            motors[i].setPower(values[i] / max);
        }
    }

    public static double round(double n, int c){
        return Math.pow(10, c) * (Math.round(n / Math.pow(10, c)));
    }

    public void stop(){
        for(int i = 0; i < 4; i++) {
            motors[i].setPower(0);
        }
    }


    //Tracker Wheels
    public void resetTrackerWheels(){
        trackerWheels.reset(backLeft.getCurrentPosition(),frontLeft.getCurrentPosition(),frontRight.getCurrentPosition());
    }

    public void updateTrackerWheels(double seconds){
        trackerWheels.update(backLeft.getCurrentPosition(),frontLeft.getCurrentPosition(),frontRight.getCurrentPosition(), seconds);
    }


  /*  @Deprecated
    public void xPID(Pos vel, double targetspd, double angle, double targretAngle){
        double error = targetspd - vel.x;
        sumError += error;
        double output = targetspd * velP + sumError * velI;

        double angleError = targretAngle - angle;


        setFromAxis(output, 0, angleError * -0.03);

        opMode.telemetry.addData("Velocity X", vel.x);
        opMode.telemetry.addData("Target Speed", targetspd);
    }

    @Deprecated
    public void xPIDwRotation(Pos vel, double targetspd, double rotatePer){
        double error = targetspd - vel.x;
        sumError += error;
        double output = targetspd * velP + sumError * velI;

        setFromAxis(output, 0, output*rotatePer);

        opMode.telemetry.addData("Velocity X", vel.x);
        opMode.telemetry.addData("Target Speed", targetspd);
    }
*/
    public void updateMovement(Pos target, MotionProfile moveProfile, MotionProfile rotProfile, double time, boolean setMotors){
        Pos delta = target.sub(trackerWheels.pos);

        opMode.telemetry.addData("Delta", delta);

        double distance = delta.getDistance();

        double moveTargetSpeed = moveProfile.getTargetSpeed(distance);
        double rotTargetSpeed = rotProfile.getTargetSpeed(delta.angle.rad());

        Pos robotDelta = delta.rotate(trackerWheels.pos.angle);

        opMode.telemetry.addData("Robot Delta", robotDelta);

        Pos move = new Pos(moveTargetSpeed, 0, new Angle());
        move = move.rotate(robotDelta.translationAngle());

        opMode.telemetry.addData("X-Vel", trackerWheels.velocity.x);
        opMode.telemetry.addData("X-Tar", move.x);
        opMode.telemetry.addData("Y-Vel", trackerWheels.velocity.y);
        opMode.telemetry.addData("Y-Tar", move.y);
        opMode.telemetry.addData("R-Vel", trackerWheels.velocity.angle.rad());
        opMode.telemetry.addData("R-Tar", rotTargetSpeed);

        if(setMotors) {
            double x = xPID.update(trackerWheels.velocity.x, move.x, time);
            double y = yPID.update(trackerWheels.velocity.y, move.y, time);
            double r = rPID.update(trackerWheels.velocity.angle.rad(), rotTargetSpeed, time);

            opMode.telemetry.addData("X OUT", x);
            opMode.telemetry.addData("Y OUT", y);
            opMode.telemetry.addData("R OUT", r);

            setFromAxis(x, y, r);
        }else{
            setFromAxis(0, 0, 0);
            xPID.resetI();
            yPID.resetI();
            rPID.resetI();
        }
    }
}
