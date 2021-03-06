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

    MotionProfile moveProfile = new MotionProfile(100, 80);
    MotionProfile rotProfile = new MotionProfile(3, 2);

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

                if(gamepad1.x){
                    mode = Mode.X;
                }
                if(gamepad1.y){
                    mode = Mode.Y;
                }
                if(gamepad1.b){
                    mode = mode.R;
                }

                if(gamepad1.right_bumper){
                    var = var.P;
                }
                if(gamepad1.left_bumper){
                    var = var.I;
                }
                if(gamepad1.right_trigger > .5){
                    var = var.D;
                }
                if(gamepad1.left_trigger > .5){
                    var = var.F;
                }

                PIDF p = dt.xPID;
                switch(mode){
                    case R:
                        p = dt.rPID;
                        break;
                    case Y:
                        p = dt.yPID;
                        break;
                }

                double change = 0;
                if(gamepad1.dpad_up){
                    change = 1;
                }else if(gamepad1.dpad_down){
                    change = -1;
                }

                switch(var){
                    case P:
                        p.p += change * 0.001; // .0001
                        break;
                    case I:
                        p.i += change * 0.0001; //.00001
                        break;
                    case D:
                        p.d += change * 0.000001;
                        break;
                    case F:
                        p.f += change * 0.0001;
                        break;
                }

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
