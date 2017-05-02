// Copyright 2016 Google Inc.

package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.adafruit.BNO055IMU.AccelUnit;
import com.qualcomm.hardware.adafruit.BNO055IMU.AccelerationIntegrator;
import com.qualcomm.hardware.adafruit.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.adafruit.BNO055IMU.Parameters;
import com.qualcomm.hardware.adafruit.BNO055IMU.SensorMode;
import com.qualcomm.hardware.adafruit.BNO055IMU.TempUnit;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.adafruit.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;

/**
 * A class that provides JavaScript access to {@link AdafruitBNO055IMU#Parameters}.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class AdafruitBNO055IMUParametersAccess extends Access {

  AdafruitBNO055IMUParametersAccess(String identifier) {
    super(identifier);
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public Parameters create() {
    try {
      return new Parameters();
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.create - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setAccelUnit(Object adafruitBNO055IMUParameters, String accelUnitString) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        AccelUnit accelUnit = AccelUnit.valueOf(accelUnitString.toUpperCase(Locale.ENGLISH));
        ((Parameters) adafruitBNO055IMUParameters).accelUnit = accelUnit;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setAccelUnit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setAccelUnit - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getAccelUnit(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        AccelUnit accelUnit = ((Parameters) adafruitBNO055IMUParameters).accelUnit;
        if (accelUnit != null) {
          return accelUnit.toString();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getAccelUnit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getAccelUnit - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setAccelerationIntegrationAlgorithm(
      Object adafruitBNO055IMUParameters, String accelerationIntegrationAlgorithm) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        accelerationIntegrationAlgorithm =
            accelerationIntegrationAlgorithm.toUpperCase(Locale.ENGLISH);
        if (accelerationIntegrationAlgorithm.equals("NAIVE")) {
          ((Parameters) adafruitBNO055IMUParameters).accelerationIntegrationAlgorithm = null;
        } else if (accelerationIntegrationAlgorithm.equals("JUST_LOGGING")) {
          ((Parameters) adafruitBNO055IMUParameters).accelerationIntegrationAlgorithm =
              new JustLoggingAccelerationIntegrator();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setAccelerationIntegrationAlgorithm - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setAccelerationIntegrationAlgorithm - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getAccelerationIntegrationAlgorithm(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        AccelerationIntegrator accelerationIntegrationAlgorithm =
            ((Parameters) adafruitBNO055IMUParameters).accelerationIntegrationAlgorithm;
        if (accelerationIntegrationAlgorithm == null ||
            accelerationIntegrationAlgorithm instanceof NaiveAccelerationIntegrator) {
          return "NAIVE";
        }
        if (accelerationIntegrationAlgorithm instanceof JustLoggingAccelerationIntegrator) {
          return "JUST_LOGGING";
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getAccelerationIntegrationAlgorithm - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getAccelerationIntegrationAlgorithm - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setAngleUnit(Object adafruitBNO055IMUParameters, String angleUnitString) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        AngleUnit angleUnit =
            AngleUnit.valueOf(angleUnitString.toUpperCase(Locale.ENGLISH));
        ((Parameters) adafruitBNO055IMUParameters).angleUnit = angleUnit;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setAngleUnit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setAngleUnit - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getAngleUnit(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        AngleUnit angleUnit = ((Parameters) adafruitBNO055IMUParameters).angleUnit;
        if (angleUnit != null) {
          return angleUnit.toString();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getAngleUnit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getAngleUnit - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setCalibrationDataFile(Object adafruitBNO055IMUParameters, String calibrationDataFile) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        ((Parameters) adafruitBNO055IMUParameters).calibrationDataFile = calibrationDataFile;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setCalibrationDataFile - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setCalibrationDataFile - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getCalibrationDataFile(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        String calibrationDataFile = ((Parameters) adafruitBNO055IMUParameters).calibrationDataFile;
        if (calibrationDataFile != null) {
          return calibrationDataFile;
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getCalibrationDataFile - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getCalibrationDataFile - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setI2cAddress7Bit(Object adafruitBNO055IMUParameters, int i2cAddr7Bit) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        ((Parameters) adafruitBNO055IMUParameters).i2cAddr = I2cAddr.create7bit(i2cAddr7Bit);
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setI2cAddress7Bit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setI2cAddress7Bit - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public int getI2cAddress7Bit(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        I2cAddr i2cAddr = ((Parameters) adafruitBNO055IMUParameters).i2cAddr;
        if (i2cAddr != null) {
          return i2cAddr.get7Bit();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getI2cAddress7Bit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getI2cAddress7Bit - caught " + e);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setI2cAddress8Bit(Object adafruitBNO055IMUParameters, int i2cAddr8Bit) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        ((Parameters) adafruitBNO055IMUParameters).i2cAddr = I2cAddr.create8bit(i2cAddr8Bit);
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setI2cAddress8Bit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setI2cAddress8Bit - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public int getI2cAddress8Bit(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        I2cAddr i2cAddr = ((Parameters) adafruitBNO055IMUParameters).i2cAddr;
        if (i2cAddr != null) {
          return i2cAddr.get8Bit();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getI2cAddress8Bit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getI2cAddress8Bit - caught " + e);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setLoggingEnabled(Object adafruitBNO055IMUParameters, boolean loggingEnabled) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        ((Parameters) adafruitBNO055IMUParameters).loggingEnabled = loggingEnabled;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setLoggingEnabled - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setLoggingEnabled - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public boolean getLoggingEnabled(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        return ((Parameters) adafruitBNO055IMUParameters).loggingEnabled;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getLoggingEnabled - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getLoggingEnabled - caught " + e);
    }
    return false;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setLoggingTag(Object adafruitBNO055IMUParameters, String loggingTag) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        ((Parameters) adafruitBNO055IMUParameters).loggingTag = loggingTag;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setLoggingTag - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setLoggingTag - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getLoggingTag(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        String loggingTag = ((Parameters) adafruitBNO055IMUParameters).loggingTag;
        if (loggingTag != null) {
          return loggingTag;
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getLoggingTag - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getLoggingTag - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setSensorMode(Object adafruitBNO055IMUParameters, String sensorModeString) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        SensorMode sensorMode =
            SensorMode.valueOf(sensorModeString.toUpperCase(Locale.ENGLISH));
        ((Parameters) adafruitBNO055IMUParameters).mode = sensorMode;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setSensorMode - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setSensorMode - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getSensorMode(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        SensorMode sensorMode = ((Parameters) adafruitBNO055IMUParameters).mode;
        if (sensorMode != null) {
          return sensorMode.toString();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getSensorMode - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getSensorMode - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setTempUnit(Object adafruitBNO055IMUParameters, String temperatureUnitString) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        TempUnit temperatureUnit =
            TempUnit.valueOf(temperatureUnitString.toUpperCase(Locale.ENGLISH));
        ((Parameters) adafruitBNO055IMUParameters).temperatureUnit = temperatureUnit;
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.setTempUnit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.setTempUnit - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getTempUnit(Object adafruitBNO055IMUParameters) {
    try {
      if (adafruitBNO055IMUParameters instanceof Parameters) {
        TempUnit temperatureUnit = ((Parameters) adafruitBNO055IMUParameters).temperatureUnit;
        if (temperatureUnit != null) {
          return temperatureUnit.toString();
        }
      } else {
        RobotLog.e("AdafruitBNO055IMUParameters.getTempUnit - " +
            "adafruitBNO055IMUParameters is not a AdafruitBNO055IMU.Parameters");
      }
    } catch (Exception e) {
      RobotLog.e("AdafruitBNO055IMUParameters.getTempUnit - caught " + e);
    }
    return "";
  }

}
