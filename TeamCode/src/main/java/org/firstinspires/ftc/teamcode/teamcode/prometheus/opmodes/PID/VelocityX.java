package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.TrackerWheels;

@TeleOp()
public class VelocityX extends LinearOpMode {

    DriveTrain dt;
    TrackerWheels tw;

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        dt = new DriveTrain(this, true);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        tw = new TrackerWheels(this);
        waitForStart();


        tw.reset(dt.backLeft.getCurrentPosition(),dt.frontLeft.getCurrentPosition(),dt.frontRight.getCurrentPosition());

        time.reset();




        while(opModeIsActive()){

            if(time.milliseconds()>1000/50) {
                tw.update(dt.backLeft.getCurrentPosition(), dt.frontLeft.getCurrentPosition(), dt.frontRight.getCurrentPosition(), time.seconds());

                if (gamepad1.left_bumper) {
                    tw.pos = new Pos();
                }
                dt.setFromAxis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

                Pos vel = tw.velocity;
                Pos accel = tw.getAcceleration(time.seconds());


                telemetry.addData("Time", time.seconds());
                time.reset();

                telemetry.addData("Velocity X", vel.x);
                telemetry.addData("Velocity Y", vel.y);
                telemetry.addData("Angle Velocity", vel.angle.getDegrees());
                telemetry.addData("Total Velocity", Math.sqrt(Math.pow(vel.y, 2) + Math.pow(vel.x, 2)));

                telemetry.addData("Acceleration X", accel.x);
                telemetry.addData("Acceleration Y", accel.y);
                telemetry.addData("Angle Acceleration", accel.angle.getDegrees());
                telemetry.addData("Total Acceleration", Math.sqrt(Math.pow(accel.x, 2) + Math.pow(accel.y, 2)));

                telemetry.update();
            }
        }
    }
}
