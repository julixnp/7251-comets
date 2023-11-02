package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTesting", group="Testing")
public class Servotesting extends OpMode {
    private Servo servo1;
    private Servo servo2;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Initialize your servo here
        servo1 = hardwareMap.servo.get("Servo1");  // replace "your_servo_name" with the actual name in your configuration
        servo2 = hardwareMap.servo.get("Servo2");
        telemetry.addData("Status", "Initialization Complete");
    }

    @Override
    public void loop() {
        // Check if the button is pressed (assuming button 'A' for example, change it as needed)
        if (gamepad1.a) {
            rotateServoClockwise();
            telemetry.addData("Status", "Rotating Servo Clockwise");
        } else {
            stopServo();
            telemetry.addData("Status", "Stopping Servo");
        }

        if (gamepad1.b) {
            rotateServoCounterClockwise();
            telemetry.addData("Status", "Rotating Servo Clockwise");
        } else {
            stopServo();
            telemetry.addData("Status", "Stopping Servo");
        }

        if (gamepad1.y) {
            servo1.setPosition(1.0);
            servo2.setPosition(1.0);
            telemetry.addData("Status", "Rotating Servo Clockwise");
        }
        if (gamepad1.x) {
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
            telemetry.addData("Status", "Rotating Servo Clockwise");
        }

            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();

        }


        private void rotateServoClockwise () {
            // Add code to rotate the servo clockwise
            servo1.setPosition(0.0);
            servo2.setPosition(0.0); // Set the servo position to rotate clockwise

        }

        private void rotateServoCounterClockwise () {
            // Add code to rotate the servo clockwise
            servo1.setPosition(1.0);
            servo2.setPosition(1.0); // Set the servo position to rotate clockwise

        }

        private void stopServo () {
            // Add code to stop the servo
            servo1.setPosition(0.5);
            servo2.setPosition(0.5); // Set the servo position to rotate clockwise

        }
    }


