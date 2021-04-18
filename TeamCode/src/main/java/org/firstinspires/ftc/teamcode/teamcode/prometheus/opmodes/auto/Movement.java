package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.MotionProfile;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.PIDF;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.TrackerWheels;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;

@TeleOp
public class Movement extends LinearOpMode {

    DriveTrain dt;
    Shooter shooter;
    TrackerWheels tw;

    Collector collector;
    WobbleArm wobbleArm;

    ElapsedTime loopTime = new ElapsedTime();

    MotionProfile moveProfile = new MotionProfile(100, 120);
    MotionProfile rotProfile = new MotionProfile(3, 4);

    enum Mode{
        X, Y, R
    }

    enum Var{
        P, I, D, F
    }

    Mode mode = Mode.X;
    Var var = Var.P;


    @Override
    public void runOpMode() throws InterruptedException {
        //FTC Dashboard
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.setAutoClear(false);


        dt = new DriveTrain(this);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new Shooter(this);

        collector = new Collector(this);

        wobbleArm = new WobbleArm(this, true);

        //log("Ready");
        //updateLog();

        waitForStart();
        loopTime.reset();
        dt.resetTrackerWheels();

        while(opModeIsActive()){
            if(loopTime.milliseconds() > 50) {
                //Update Tracker Wheels

                dt.updateConstants();



                telemetry.addLine(mode.toString());
                telemetry.addLine(var.toString());
                telemetry.addLine("X PID");
                telemetry.addLine(dt.xPID.toString());
                telemetry.addLine("Y PID");
                telemetry.addLine(dt.yPID.toString());
                telemetry.addLine("R PID");
                telemetry.addLine(dt.rPID.toString());

                dt.updateTrackerWheels(loopTime.seconds());


                dt.updateMovement(new Pos(0, 0, new Angle()), moveProfile, rotProfile, loopTime.seconds(), gamepad1.a);


                loopTime.reset();

                telemetry.update();
            }
        }
    }
}
