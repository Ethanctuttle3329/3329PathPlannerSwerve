package frc.robot.subsystems.Swerve;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase{
    private final SwerveDrive swerveDrive;

    public Swerve(File directory)
    {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
        System.out.println("\"conversionFactors\": {");
        System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
        System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
        System.out.println("}");
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();
        setupPathplanner();
    }

    @Override
    public void periodic()
    {

    }

    @Override
    public void simulationPeriodic()
    {
    }

    public void setupPathplanner()
    {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                AutonConstants.TRANSLATION_PID,
                AutonConstants.ANGLE_PID,
                4.5,
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()
        ),
        () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this);
    }

    public Command getAutonomousCommand(String pathName)
    {
        return new PathPlannerAuto(pathName);
    }

    public Command driveToPose(Pose2d pose)
    {
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumVelocity(), 4.0, 
            swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            0.0,
            0.0
        );
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
    {
        return run(() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                            headingX.getAsDouble(),
                                                                            headingY.getAsDouble(),
                                                                            swerveDrive.getOdometryHeading().getRadians(),
                                                                            swerveDrive.getMaximumVelocity()));
        });
    }

    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
    {
        return run(() -> {
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                            translationY.getAsDouble(),
                                                                            rotation.getAsDouble() * Math.PI,
                                                                            swerveDrive.getOdometryHeading().getRadians(),
                                                                            swerveDrive.getMaximumVelocity()));
        });
    }

    public Command sysIdDriveMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                new Config(),
                this, swerveDrive, 12),
            3.0, 5.0, 3.0);
    }

    public Command sysIdAngleMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                new Config(),
                this, swerveDrive),
            3.0, 5.0, 3.0);
    }

    public Command centerModulesCommand()
    {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                               .forEach(it -> it.setAngle(0.0)));
    }

    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
    {
        return Commands.deferredProxy(
            () -> Commands.run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)), this)
                      .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                                   distanceInMeters)
                                 );
    }

    public void setMaximumSpeed(double maximumSpeedInMetersPerSecond)
    {
        swerveDrive.setMaximumSpeed(maximumSpeedInMetersPerSecond,
                                    false,
                                    swerveDrive.swerveDriveConfiguration.physicalCharacteristics.optimalVoltage);
    }
    
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
    {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
          swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), 0.8),
                            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                            true,
                            false);
        });
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                          rotation,
                          fieldRelative,
                          false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void drive(ChassisSpeeds velocity)
    {
      swerveDrive.drive(velocity);
    }

    public SwerveDriveKinematics getKinematics()
    {
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory)
    {
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void zeroGyroWithAlliance()
    {
        if (isRedAlliance())
        {
          zeroGyro();
          resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else
        {
          zeroGyro();
        }
    }

    public void setMotorBrake(boolean brake)
    {
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                            scaledInputs.getY(),
                                                            headingX,
                                                            headingY,
                                                            getHeading().getRadians(),
                                                            Constants.MAX_SPEED);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                            scaledInputs.getY(),
                                                            angle.getRadians(),
                                                            getHeading().getRadians(),
                                                            Constants.MAX_SPEED);
    }

    public ChassisSpeeds getFieldVelocity()
    {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController()
    {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock()
    {
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch()
    {
        return swerveDrive.getPitch();
    }
}