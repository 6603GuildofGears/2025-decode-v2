package org.firstinspires.ftc.teamcode.pedroPathing.Autos.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;  
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;


@Autonomous(name="Move", group="Auto")
public class move extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        intMotors(this);
        
        telemetry.addData("Status", "Ready to move forward");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Move forward at half speed
            frontLeft.setPower(-0.75);
            frontRight.setPower(-0.75);
            backLeft.setPower(-0.75);
            backRight.setPower(-0.75);
            
            sleep(3000); // Move for 2 seconds
            
            // Stop all motors
            frontLeft.setPower(0);  
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            
            telemetry.addData("Status", "Move complete");
            telemetry.update();
        }
    }
}