package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyro; //gyro that helps with field centric mode

    private SwerveDriveOdometry swerveOdometry;
    private frc.robot.subsystems.SwerveModule[] mSwerveMods;

    private Field2d field;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        zeroGyro();
        
        //swerve modules
        mSwerveMods = new frc.robot.subsystems.SwerveModule[] {
                new frc.robot.subsystems.SwerveModule(0, Constants.Swerve.Mod0.constants),
                new frc.robot.subsystems.SwerveModule(1, Constants.Swerve.Mod1.constants),
                new frc.robot.subsystems.SwerveModule(2, Constants.Swerve.Mod2.constants),
                new frc.robot.subsystems.SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        field = new Field2d();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Gyro", gyro);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getStates());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drive(
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
             fieldRelative, isOpenLoop
        );
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw())
            :speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    //stops swerve motion
    public void stop() {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, getYaw()));
        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getStates(), pose);
    }

    //returns position of each module
    public SwerveModulePosition[] getStates() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getState();
        }
        return positions;
    }

    //sets gyro to zero
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    //sets gyro to a specific value
    public void setGyro(double value) {
        gyro.setYaw(value);
    }

    //returns yaw of gyro
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - (gyro.getYaw().getValue()))
                : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    //updates information of SmartDashboard
    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getStates());
        field.setRobotPose(getPose());

        for (frc.robot.subsystems.SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " NEO Position", mod.getAngle().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().distanceMeters);
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Module Angle", mod.getState().angle.getDegrees());
        }
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        (speeds) -> drive(speeds, false, false),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD),
            new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD)),
        new RobotConfig(0, 0, null, 0),
        () -> {
            var alliance = DriverStation.getAlliance();
            if(alliance.isPresent()){
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
    ); 
}