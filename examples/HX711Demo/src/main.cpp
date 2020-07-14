#include <Arduino.h>

#include <HX711.h>

HX711 scale(A1, A0);

#define TARE_AVG        20
#define MEAS_AVG        1
#define SCALE           1.0

void setup() 
{
    Serial.begin(115200);

    scale.begin();
    scale.set_gain(HX711::GAIN_128);
    // scale.set_scale(SCALE);

    // scale.tare(TARE_AVG);
};

void loop() 
{
    // long raw = scale.read_average(MEAS_AVG);
    long raw = scale.read();
    // float weight = scale.get_units(MEAS_AVG);
    Serial.println(raw);

    // Serial.print(", ");
    // Serial.println(weight);    

    delay(10);
};
