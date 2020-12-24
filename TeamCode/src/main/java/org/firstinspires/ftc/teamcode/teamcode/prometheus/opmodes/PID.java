package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PID {
    public class DriveTrain {

        public DcMotor frontRight;
        public DcMotor frontLeft;
        public DcMotor backRight;
        public DcMotor backLeft;

        public DcMotor[] motors;

        private OpMode opMode;

        private boolean tel;

        public DriveTrain(OpMode opMode, boolean tel){
            this.opMode = opMode;
            this.tel = tel;

            frontRight = opMode.hardwareMap.get(DcMotor.class, "fr");
            frontLeft = opMode.hardwareMap.get(DcMotor.class, "fl");
            backRight = opMode.hardwareMap.get(DcMotor.class, "br");
            backLeft = opMode.hardwareMap.get(DcMotor.class, "bl");

            motors = new DcMotor[]{frontRight, frontLeft, backLeft, backRight};
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
                if(num > max){
                    max = num;
                }
            }

            for(int i = 0; i < 4; i++) {
                motors[i].setPower(values[i] / max);
            }

            if (tel) {
                opMode.telemetry.addData("Drive", "(" + round(x, 2) + ", " + round(y, 2) + ") - " + round(r, 2));
            }
        }

        public double round(double n, int c){
            return Math.pow(10, c) * (Math.round(n / Math.pow(10, c)));
        }
    }

}

