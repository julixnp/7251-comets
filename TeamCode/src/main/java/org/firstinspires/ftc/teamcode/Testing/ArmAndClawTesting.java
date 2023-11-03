package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTesting", group="Testing")
public class ArmAndClawTesting extends LinearOpMode {
    private Servo servo1;

    private DcMotor motor1;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {


        // Initialize your servo here
        servo1 = hardwareMap.servo.get("Servo1");  // replace "your_servo_name" with the actual name in your configuration
        motor1 = hardwareMap.dcMotor.get("Motor1");

        telemetry.addData("Status", "Initialization Complete");
        waitForStart();
        
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (opModeIsActive()) {
            // Servo Code
            if (gamepad1.right_bumper) {
                servo1.setPosition(0.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            } else {
                servo1.setPosition(0.5);
                telemetry.addData("Status", "Stopping Servo");
            }

            if (gamepad1.left_bumper) {
                servo1.setPosition(1.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            } else {
                servo1.setPosition(0.5);
                telemetry.addData("Status", "Stopping Servo");
            }

            if (gamepad1.dpad_up) {
                servo1.setPosition(1.0);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            }
            if (gamepad1.dpad_down) {
                servo1.setPosition(0.5);
                telemetry.addData("Status", "Rotating Servo Clockwise");
            }


            //Arm code
            double power = 1;
            if (gamepad1.a) {
                motor1.setTargetPosition(0);
                motor1.setPower(power);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.left_stick_y * -1 > 0) {
                motor1.setTargetPosition(motor1.getCurrentPosition() + 300);
                motor1.setPower(power);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.left_stick_y * -1 < 0) {
                motor1.setTargetPosition(motor1.getCurrentPosition() - 300);
                motor1.setPower(power);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();

        }
    }
}


