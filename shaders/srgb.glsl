float linear2srgb(float v) {
    if (v > 0.0) {
        if (v <= 0.0031308)
            return v * 12.92;
        else
            return pow(v, 1.0 / 2.4) * 1.055 - 0.055;
    } else {
        return 0.0;
    }
}

float srgb2linear(float srgb) {
  if (srgb < 0.04045) {
    return srgb * (25.0 / 232.0);
  } else {
    return pow((200.0 * srgb + 11.0) * (1.0 / 211.0), 12.0 / 5.0);
  }
}