// Copyright 2016 Google Inc.

package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.util.HardwareItem;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU.CalibrationStatus;
import com.qualcomm.hardware.adafruit.BNO055IMU.Parameters;
import com.qualcomm.hardware.adafruit.BNO055IMU.SystemError;
import com.qualcomm.hardware.adafruit.BNO055IMU.SystemStatus;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;
import junit.framework.Assert;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

/**
 * A class that provides JavaScript access to {@link AdafruitBNO055IMU}.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class AdafruitBNO055IMUAccess extends HardwareAccess<AdafruitBNO055IMU> {
  private final AdafruitBNO055IMU imu;

  AdafruitBNO055IMUAccess(
      HardwareItem hardwareItem, HardwareMap hardwareMap, Class<? extends HardwareDevice> deviceType) {
    super(hardwareItem, hardwareMap, AdafruitBNO055IMU.class);
    Assert.assertTrue(deviceType == AdafruitBNO055IMU.class);
    this.imu = hardwareDevice;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Acceleration getAcceleration() {
    try {
      if (imu != null) {
        return imu.getAcceleration();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getAcceleration - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Orientation getAngularOrientation() {
    try {
      if (imu != null) {
        return imu.getAngularOrientation();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getAngularOrientation - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public AngularVelocity getAngularVelocity() {
    try {
      if (imu != null) {
        return imu.getAngularVelocity();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getAngularVelocity - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getCalibrationStatus() {
    try {
      if (imu != null) {
        CalibrationStatus calibrationStatus = imu.getCalibrationStatus();
        if (calibrationStatus != null) {
          return calibrationStatus.toString();
        }
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getCalibrationStatus - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Acceleration getGravity() {
    try {
      if (imu != null) {
        return imu.getGravity();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getGravity - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Acceleration getLinearAcceleration() {
    try {
      if (imu != null) {
        return imu.getLinearAcceleration();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getLinearAcceleration - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public MagneticFlux getMagneticFieldStrength() {
    try {
      if (imu != null) {
        return imu.getMagneticFieldStrength();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getMagneticFieldStrength - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Acceleration getOverallAcceleration() {
    try {
      if (imu != null) {
        return imu.getOverallAcceleration();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getOverallAcceleration - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Parameters getParameters() {
    try {
      if (imu != null) {
        return imu.getParameters();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getParameters - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Position getPosition() {
    try {
      if (imu != null) {
        return imu.getPosition();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getPosition - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Quaternion getQuaternionOrientation() {
    try {
      if (imu != null) {
        return imu.getQuaternionOrientation();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getQuaternionOrientation - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getSystemError() {
    try {
      if (imu != null) {
        SystemError systemError = imu.getSystemError();
        if (systemError != null) {
          return systemError.toString();
        }
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getSystemError - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getSystemStatus() {
    try {
      if (imu != null) {
        SystemStatus systemStatus = imu.getSystemStatus();
        if (systemStatus != null) {
          return systemStatus.toString();
        }
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getSystemStatus - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Temperature getTemperature() {
    try {
      if (imu != null) {
        return imu.getTemperature();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getTemperature - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Velocity getVelocity() {
    try {
      if (imu != null) {
        return imu.getVelocity();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.getVelocity - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void initialize(Object parameters) {
    try {
      if (imu != null) {
        if (parameters instanceof Parameters) {
          imu.initialize(((Parameters) parameters));
        } else {
          RobotLog.e("AdafruitBNO055IMU.initialize - " +
              "parameters is not a AdafruitBNO055IMU.Parameters");
        }
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.initialize - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void startAccelerationIntegration_with1(int msPollInterval) {
    try {
      if (imu != null) {
        imu.startAccelerationIntegration(
            null /* initialPosition */, null /* initialVelocity*/, msPollInterval);
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.startAccelerationIntegration - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void startAccelerationIntegration_with3(Position initialPosition, Velocity initialVelocity, int msPollInterval) {
    try {
      if (imu != null) {
        if (initialPosition instanceof Position) {
          if (initialVelocity instanceof Velocity) {
            imu.startAccelerationIntegration(
                (Position) initialPosition, (Velocity) initialVelocity, msPollInterval);
          } else {
            RobotLog.e(
                "AdafruitBNO055IMU.startAccelerationIntegration - initialVelocity is not a Velocity");
          }
        } else {
          RobotLog.e(
              "AdafruitBNO055IMU.startAccelerationIntegration - initialPosition is not a Position");
        }
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.startAccelerationIntegration - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void stopAccelerationIntegration() {
    try {
      if (imu != null) {
        imu.stopAccelerationIntegration();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.stopAccelerationIntegration - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public boolean isSystemCalibrated() {
    try {
      if (imu != null) {
        return imu.isSystemCalibrated();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.isSystemCalibrated - caught " + e);
    }
    return false;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public boolean isGyroCalibrated() {
    try {
      if (imu != null) {
        return imu.isGyroCalibrated();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.isGyroCalibrated - caught " + e);
    }
    return false;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public boolean isAccelerometerCalibrated() {
    try {
      if (imu != null) {
        return imu.isAccelerometerCalibrated();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.isAccelerometerCalibrated - caught " + e);
    }
    return false;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public boolean isMagnetometerCalibrated() {
    try {
      if (imu != null) {
        return imu.isMagnetometerCalibrated();
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.isMagnetometerCalibrated - caught " + e);
    }
    return false;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void saveCalibrationData(String absoluteFileName) {
    try {
      if (imu != null) {
        ReadWriteFile.writeFile(
            AppUtil.getInstance().getSettingsFile(absoluteFileName),
            imu.readCalibrationData().serialize());
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMU.saveCalibrationData - caught " + e);
    }
  }
}
