// IMU
struct ImuData {
    ImuData()
    {
        valid = true;
        head = pitch = roll = 999.0;
        headRot = pitchRot = rollRot = 999.0;
        headAcc = pitchAcc = rollAcc = 999.0;
        temp						 = 999.0;
        MagneticX = MagneticY = MagneticZ = 999.0;
        Quaternion0 = Quaternion1 = Quaternion2 = Quaternion3 = 999.0;
    }
    bool   valid;
    double head;
    double pitch;
    double roll;
    double headRot;
    double pitchRot;
    double rollRot;
    double headAcc;
    double pitchAcc;
    double rollAcc;
    double MagneticX;
    double MagneticY;
    double MagneticZ;
    double Quaternion0;
    double Quaternion1;
    double Quaternion2;
    double Quaternion3;

    double temp;
};