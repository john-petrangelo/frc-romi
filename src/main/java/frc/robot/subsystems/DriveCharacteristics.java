package frc.robot.subsystems;

class DriveCharacteristics {
    public final double kSLeftFwd;
    public final double kVLeftFwd;

    public final double kSLeftBack;
    public final double kVLeftBack;

    public final double kSRightFwd;
    public final double kVRightFwd;

    public final double kSRightBack;
    public final double kVRightBack;

    DriveCharacteristics(double kSLeftFwd, double kVLeftFwd,
                    double kSLeftBack, double kVLeftBack,
                    double kSRightFwd, double kVRightFwd,
                    double kSRightBack, double kVRightBack) {
      this.kSLeftFwd = kSLeftFwd;
      this.kVLeftFwd = kVLeftFwd;

      this.kSLeftBack = kSLeftBack;
      this.kVLeftBack = kVLeftBack;
  
      this.kSRightFwd = kSRightFwd;
      this.kVRightFwd = kVRightFwd;

      this.kSRightBack = kSRightBack;
      this.kVRightBack = kVRightBack;
    }

    @Override
    public String toString() {
        return "DriveCharacteristics Left=[kSFwd=" + kSLeftFwd
          + ", kVFwd=" + kVLeftFwd 
          + ", kSBack=" + kSLeftBack
          + ", kVBack=" + kVLeftBack
          + "], Right=[kSFwd=" + kSRightFwd 
          + ", kVFwd=" + kVRightFwd
          + ", kSBack=" + kSRightBack
          + ", kVBack=" + kVRightBack + "]";
    }
  }
  