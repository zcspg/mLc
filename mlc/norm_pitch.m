function X = norm_pitch(pitch)
    while(pitch < -90)
        pitch = -180 - pitch;
    end
    while(pitch > 90)
        pitch = 180 - pitch;
    end
    X = pitch;