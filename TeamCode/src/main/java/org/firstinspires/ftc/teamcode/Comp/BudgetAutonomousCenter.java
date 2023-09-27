package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareBudgetRobot;

public class BudgetAutonomousCenter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double    COUNTS_PER_MOTOR_REV    = 384.5;

    static final double    DRIVE_GEAR_REDUCTION    = 1 ;

    static final double    WHEEl_DIAMETER_INCHES   = 4.0 ;

    static final double    COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEl_DIAMETER_INCHES * 3.1415);

    static final double    DRIVE_SPEED             = 0.6;

    static double targetPosition;

    HardwareBudgetRobot robot= new HardwareBudgetRobot(this );

    static final double FEET_PER_METER = 3.28084;

    static  double targetPosition;

    HardwareBudgetRobot robot = new HardwareBudgetRobot(this);

    //All units are METERS
    @Override
    public void runOpMode()
    {
        //Robot and Dash
        robot.init();
        FtcDashboard dashboard = FtcDashboard.getInstance();
    }

    public void strafeDrive(double speed, double target){
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newMotor1Target = robot.motor1.getCurrentPosition() + (int) target * COUNTS_PER_INCH);
        newMotor2Target = robot.motor2.getCurrentPosition() + (int) target * COUNTS_PER_INCH);
        newMotor3Target = robot.motor3.getCurrentPosition() + (int) target * COUNTS_PER_INCH);
        newMotor4Target = robot.motor4.getCurrentPosition() + (int) target * COUNTS_PER_INCH);

        //Ensure that the opmode is still active
        if(opModeIsActive()){

            robot.motor1.setTargetPosition(newMotor1Target);
            robot.motor2.setTargetPosition(-1*newMotor1Target);
            robot.motor3.setTargetPosition(-1*newMotor1Target);
            robot.motor4.setTargetPosition(newMotor1Target);

            robot.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //reset the timeout time and start motion. (these values are for strafing right)
            runtime.reset();


            frontLeftPower = speed;
            frontRightPower = speed;
            backLeftPower = speed;
            backRightPower = speed;

            while (opModeIsActive() &&
                    (robot.motor1.isBusy() && robot.motor2.isBusy()) && (robot.motor3.isBusy() && robot.motor3.isBusy())) {

                //Display it for the driver
                telemetry.addData("Running to (LINEAR DRIVE)"," Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target);
                telemetry.addData("Currently at (LINEAR DRIVE)",  "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        robot.motor1.getCurrentPosition(), robot.motor2.getCurrentPosition(), robot.motor3.getCurrentPosition(), robot.motor4.getCurrentPosition());
                telemetry.update();
            }

            //Stop all motion;
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
            robot.motor3.setPower(0);
            robot.motor4.setPower(0);


        }


    }

}