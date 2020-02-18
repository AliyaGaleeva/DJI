package test.lab.vision;

public class MissionCoordinate {

    private float mPitch = 0f; // вперед (реально летит в сторону)
    private float mRoll = 0f; // в сторону (реально летит вперед)
    private float mYaw = 0f; // поворот
    private float mThrottle = TrackingActivity.DRONE_MISSION_HEIGHT; // вверх
    private String course = "неизвестное значение";

    public float getPitch() {
        return mPitch;
    }

    public void setPitch(float pitch) {
        mPitch = pitch;
    }

    public float getRoll() {
        return mRoll;
    }

    public void setRoll(float roll) {
        mRoll = roll;
    }

    public float getYaw() {
        return mYaw;
    }

    public void setYaw(float yaw) {
        mYaw = yaw;
    }

    public float getThrottle() {
        return mThrottle;
    }

    public void setThrottle(float throttle) {
        mThrottle = throttle;
    }

    public String getCourse() {
        if (mPitch > 0f && mRoll == 0f){
            return "вправо";
        }else if(mPitch < 0f && mRoll == 0f){
            return "влево";
        }else if(mRoll > 0f && mPitch == 0){
            return "вперед";
        }else if (mRoll < 0f && mPitch == 0){
            return "назад";
        }else{
            return course;
        }
    }

    public void setCourse(String course) {
        this.course = course;
    }
}
