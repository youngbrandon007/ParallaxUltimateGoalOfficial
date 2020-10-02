package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp
public class Test extends LinearOpMode {

    DcMotor left;
    DcMotor right;

    DistanceSensor dist;
    ColorSensor color;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime i2cTime = new ElapsedTime();
    ElapsedTime imuTime = new ElapsedTime();

    double i2c = 0;
    double imuT = 0;

    double mm = 0;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "l");
        right = hardwareMap.get(DcMotor.class, "r");

        dist = hardwareMap.get(DistanceSensor.class, "cd");
        color = hardwareMap.get(ColorSensor.class, "cd");

        servo = hardwareMap.get(Servo.class, "s");

        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();

        telemetry.addAction(new Runnable() { @Override public void run() {
                mm = dist.getDistance(DistanceUnit.MM);
                i2c = i2cTime.milliseconds();
                i2cTime.reset();
                }
            });

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            imuT = imuTime.milliseconds();
            imuTime.reset();
        }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });



        while(opModeIsActive()){
            float forward = gamepad1.left_stick_y;
            float turn = gamepad1.right_stick_x;
            left.setPower(-(forward - turn));
            right.setPower(forward + turn);

            float ser = gamepad1.right_trigger - gamepad1.left_trigger;
            servo.setPosition((ser) / 2.0 + 0.5);


            telemetry.addData("Dist", mm);
            //telemetry.addData("a", color.alpha());
            //telemetry.addData("r", color.red());
            //telemetry.addData("g", color.green());
            //telemetry.addData("b", color.blue());
            telemetry.addData("i2c Time", i2c);
            telemetry.addData("Imu Time", imuT);
            telemetry.addData("Time", time.milliseconds());
            time.reset();
            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
