package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.71)
            .forwardZeroPowerAcceleration(-41.442107879168766)
            .lateralZeroPowerAcceleration(-59.49823903490934)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1125, 0, 0.01, 0.055))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.3, 0.0625))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.075, 0, 0.0003, 0.6, 0.015))
            .centripetalScaling(0.0009);
    
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, // T Value constraint 
            100, // timeoutConstraint 
            1.1, //breaking strength
            1 // breaking start 
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftFrontMotorName("frontLeftDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.REVERSE)
            .xVelocity(83.28172915000616)
            .yVelocity(60.98097337137057)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED) // is x
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED) // is y
            .forwardPodY(5.875)
            .strafePodX(8);



}
