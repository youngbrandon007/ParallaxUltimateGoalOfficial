package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class htmlTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        telemetry.addLine("<h1>Heading</h1>");
        telemetry.addLine("<p>contents</p>");
        //telemetry.addLine("<p id=\"test\" style=\"color: red;\">test</p>");
        telemetry.addLine("<p>\n" +
                "<span style=\"color: #ff0000\">███████████████████████████████████\n█</span>" +
                "</p>");
        telemetry.update();
        waitForStart();
    }
}
