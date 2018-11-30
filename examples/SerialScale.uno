#include <Arduino.h>

#include <HX711.h>

HX711 scale(A1, A0);

#define TARE_AVG        20
#define MEAS_AVG        5
#define SCALE           1.0

void setup() 
{
    Serial.begin(115200);

    scale.begin();
    scale.set_gain(HX711::GAIN_128);
    scale.set_scale(SCALE);

    scale.tare(TARE_AVG);
}

void loop() 
{
    long raw = scale.get_value(1);
    float weight = scale.get_units(MEAS_AVG);
    Serial.print(raw);

    Serial.print(", ");
    Serial.println(weight);    

    delay(99);
}
