package org.firstinspires.ftc.teamcode.teamcode.prometheus.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.TrackerWheels;

@TeleOp()
@Disabled
public class EncoderPosition extends LinearOpMode {

    DriveTrain dt;
    TrackerWheels tw;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(this);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        tw = new TrackerWheels(this);
        waitForStart();


        tw.reset(dt.backLeft.getCurrentPosition(),dt.frontLeft.getCurrentPosition(),dt.frontRight.getCurrentPosition());

        while(opModeIsActive()){
            tw.update(dt.backLeft.getCurrentPosition(),dt.frontLeft.getCurrentPosition(),dt.frontRight.getCurrentPosition(), 0.01);

            if(gamepad1.left_bumper){
                tw.pos = new Pos();
            }
            dt.setFromAxis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            telemetry.addData("BL",dt.backLeft.getCurrentPosition() );
            telemetry.addData("BR",dt.backRight.getCurrentPosition() );
            telemetry.addData("FR",dt.frontRight.getCurrentPosition() );
            telemetry.addData("FL",dt.frontLeft.getCurrentPosition() );

            telemetry.update();

        }
    }
}
