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
public class TrackerTuning extends LinearOpMode {

    DriveTrain dt;
    Shooter shooter;
    TrackerWheels tw;

    Collector collector;
    WobbleArm wobbleArm;

    ElapsedTime loopTime = new ElapsedTime();

    MotionProfile moveProfile = new MotionProfile(60, 40);
    MotionProfile rotProfile = new MotionProfile(2, 2);


    double totalForward = 0;
    double totalRotation = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //FTC Dashboard
        //telemetry = FtcDashboard.getInstance().getTelemetry();
        //telemetry.setAutoClear(false);


        dt = new DriveTrain(this);
        //dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

                dt.updateTrackerWheels(loopTime.seconds());

                //dt.updateMovement(new Pos(0,0,new Angle()), moveProfile, rotProfile, loopTime.seconds(), gamepad1.a);

                dt.setFromAxis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.75);

                totalForward += dt.trackerWheels.disX;
                totalRotation += dt.trackerWheels.rot;

                telemetry.addData("Forward", totalForward);
                telemetry.addData("Total Rotation", totalRotation);

                loopTime.reset();

                telemetry.update();
            }
        }
    }
}
