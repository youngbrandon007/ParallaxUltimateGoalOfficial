package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.TrackerWheels;

@TeleOp()

public class EncoderPosition extends LinearOpMode {

    DriveTrain dt;
    TrackerWheels tw;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(this, true);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        tw = new TrackerWheels(this);
        waitForStart();

        while(opModeIsActive()){
            int encX1 = dt.backLeft.getCurrentPosition();
            int encX2 = dt.frontLeft.getCurrentPosition();
            int encY = dt.frontRight.getCurrentPosition();
            tw.update(encX1,encX2,encY);

            dt.setFromAxis(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            telemetry.addData("BL",dt.backLeft.getCurrentPosition() );
            telemetry.addData("BR",dt.backRight.getCurrentPosition() );
            telemetry.addData("FR",dt.frontRight.getCurrentPosition() );
            telemetry.addData("FL",dt.frontLeft.getCurrentPosition() );

            telemetry.update();

        }
    }
}
