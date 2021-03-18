package org.firstinspires.ftc.teamcode.teamcode.prometheus.robot;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Angle;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.lib.Pos;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
@TeleOp(name = "Save/Get File")

public class SaveGetFile extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.x == true) {
                savePosition(new Pos(3,5,new Angle(7)));
            }

            if (gamepad1.b == true) {
                Pos p = getPosition();
                telemetry.addData("data", p.toString());
                telemetry.update();
            }
        }
    }
    public static void savePosition(Pos p) {
        String position = p.x + "," + p.y + "," + p.angle.getRadians();
        saveFile(position);
    }

    public static Pos getPosition() {
        String oldPosition = getFile();
        String positionArray[] = oldPosition.split(",");
            double getX = Double.parseDouble(positionArray[0]);
            double getY = Double.parseDouble(positionArray[1]);
            double getAngle = Double.parseDouble(positionArray[2]);
        Pos p = new Pos(getX, getY, new Angle(getAngle));
        return p;
    }

    public static void saveFile(String positionText) {
        try {
            File robotPositionFile = new File(Environment.getExternalStorageDirectory(), "TestFile.txt");
            if (!robotPositionFile.exists())
                robotPositionFile.createNewFile();

            // Adds a line to the file
            BufferedWriter writer = new BufferedWriter(new FileWriter(robotPositionFile, false /*not appended*/));
            writer.write(positionText);
            writer.close();
        } catch (IOException e) {
            Log.e("ReadWriteFile", "Unable to write to the TestFile.txt file.");
        }
    }

    public static String getFile() {
        String textFromFile = "";
// Gets the file from the primary external storage space of the
// current application.
        File robotPositionFile = new File(Environment.getExternalStorageDirectory(), "TestFile.txt");
        if (robotPositionFile != null) {
            StringBuilder stringBuilder = new StringBuilder();
            // Reads the data from the file
            BufferedReader reader = null;
            try {
                reader = new BufferedReader(new FileReader(robotPositionFile));
                String line;

                while ((line = reader.readLine()) != null) {
                    textFromFile += line.toString();
                    textFromFile += "\n";
                }
                reader.close();
            } catch (Exception e) {
                Log.e("ReadWriteFile", "Unable to read the TestFile.txt file.");
            }
        }
        return textFromFile;
    }
}

