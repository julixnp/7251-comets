package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="ArmTesting", group="Testing")
//@Disabled
public class ArmTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor arm;
    CRServo hand;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        hand = hardwareMap.get(CRServo.class, "Hand");
        waitForStart();

        double power = .3;
        //If Y is pressed middle height
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()) {

        }


    }

}
