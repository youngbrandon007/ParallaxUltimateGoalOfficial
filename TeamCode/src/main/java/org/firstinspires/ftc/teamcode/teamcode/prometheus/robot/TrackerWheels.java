package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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


    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    double prevAngle = 0;
    public Angle robotAngle = new Angle();

    public double disX = 0;
    public double rot = 0;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    public double getAngle(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        return angles.firstAngle;
    }

    public Angle updateAngle(){
        double a = getAngle();

        double change = a - prevAngle;

        change += (change > 180) ? -360 : (change < -180) ? 360 : 0;

        robotAngle.setDegrees(robotAngle.getDegrees() +  change);

        prevAngle = a;

        return robotAngle;
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
        updateAngle();
        opMode.telemetry.addData("Angle", robotAngle.getDegrees());

        oldPos = pos.copy();
        x1encoder.update(encX1);
        x2encoder.update(encX2);
        y1encoder.update(encY);
        double x1;
        double x2;
        double y;
        double distanceX;
        double distanceY;
        double totalCir = 9.165*Math.PI;

        x1 = x1encoder.distance();
        x2 = x2encoder.distance();
        y = y1encoder.distance();

        distanceX = ((x1+x2)/2);
        disX = distanceX;
        rot = (x1-x2) / Math.PI;

        Angle tTurn = new Angle().setDegrees((((x1-x2)/totalCir)*360)/2);

        opMode.telemetry.addData("TTURN", tTurn.getDegrees());

        distanceY = y;

        Angle turn  = new Angle(robotAngle.getRadians() - oldPos.angle.getRadians());
        Angle finalAngle = robotAngle.copy();
        //Angle turn = new Angle().setDegrees((((x1-x2)/totalCir)*360)/2);
        //Angle finalAngle = new Angle().setDegrees(pos.angle.getDegrees()+turn.getDegrees());
        Angle change = new Angle().setDegrees((pos.angle.getDegrees()+finalAngle.getDegrees())/2);

        pos.x += (distanceX * Math.cos(change.getRadians())) -  Math.sin(change.getRadians())*distanceY;
        pos.y += (distanceY * Math.cos(change.getRadians())) +  Math.sin(change.getRadians())*distanceX;
        pos.angle = robotAngle.copy();

        oldVelocity = velocity.copy();
        velocity = new Pos();
        velocity.x = distanceX / time;
        velocity.y = distanceY / time;
        velocity.angle.setDegrees(turn.getDegrees() / time);

        opMode.telemetry.addData("Position",pos);

    }
}


