#include<config.h>

#if defined(USE_MQ7_SENSOR)

    #include <math.h>

    //========================= MQ-7 Class Definition =============================//
    class MQ7 {
    private:
        uint8_t pin;
        float RL;       // Load resistor in kOhm
        float Ro;       // Clean air resistance ratio
        float a, b;     // Regression constants
        int samples;    // Number of samples for averaging

        // Private function: calculates CO ppm and averages internal data
        float calculateCO() {
            float sumVoltage = 0;
            float sumRs = 0;
            float sumCO = 0;

            for (int i = 0; i < samples; i++) {
                float voltage = readVoltage();
                float Rs = readRs(voltage);
                float RsRo = Rs / Ro;
                float CO_ppm = (RsRo > 0) ? a * pow(RsRo, b) : 0;

                sumVoltage += voltage;
                sumRs += Rs;
                sumCO += CO_ppm;

                delay(100); // Small delay between samples
            }

            avgVoltage = sumVoltage / samples;
            avgRs = sumRs / samples;
            avgRsRo = avgRs / Ro;

            return sumCO / samples;
        }

        // Private getters for internal calculation
        float readVoltage() {
            int adc = analogRead(pin);
            return adc * (3.6 / 4095.0);
        }

        float readRs(float voltage) {
            return (voltage <= 0.01) ? 0 : ((3.6 - voltage) / voltage) * RL;
        }

        // Internal storage of last averaged values
        float avgVoltage = 0;
        float avgRs = 0;
        float avgRsRo = 0;

    public:
        // Constructor with initializer list
        MQ7(uint8_t analogPin, float loadResistor = 10.0, float cleanAirRo = 27.5,
            float regA = 99.042, float regB = -1.518, int numSamples = 10)
            : pin(analogPin), RL(loadResistor), Ro(cleanAirRo),
                a(regA), b(regB), samples(numSamples)
        {
            analogSetAttenuation(ADC_11db); // ESP32 full 0–3.3V range
        }

        // Public function: only function the user calls
        float readCO() {
            return calculateCO();
        }

        // Optional: getters for last averaged internal data
        float getAvgVoltage() { return avgVoltage; }
        float getAvgRs()      { return avgRs; }
        float getAvgRsRo()    { return avgRsRo; }
    };
    //========================= End Class =============================//

    // MQ7 mq7(34); // MQ-7 analog pin GPIO34

    // void setup() {
    //     Serial.begin(115200);
    //     delay(2000);
    //     Serial.println("MQ-7 Flying Fish Module OOP Example");
    // }

    // void loop() {
    //     float co = mq7.readCO();  // Clean call, only public function

    //     Serial.println("CO ppm: "); Serial.print(co, 2);

    //     delay(2000);
    // }
#endif

#if defined(USE_NTC_SENSOR)

    #include <math.h>
    //========================= NTC Class Definition =============================//
    class NTC {
    private:
        uint8_t pin;
        float adcMax;
        float vRef;
        float seriesResistor;
        float nominalRes;
        float nominalTemp;
        float beta;
        int sampleCount;
        float offsetTemp;

        // Internal storage of last averaged values
        float avgVoltage = 0;
        float avgResistance = 0;

        // Private: read raw ADC voltage
        float readVoltage() {
        uint32_t adcSum = 0;
        for (int i = 0; i < sampleCount; i++) {
            adcSum += analogRead(pin);
            delay(5);
        }
        float adcAvg = adcSum / (float)sampleCount;
        avgVoltage = (adcAvg / adcMax) * vRef;
        return avgVoltage;
        }

        // Private: calculate NTC resistance
        float readResistance(float voltage) {
        avgResistance = seriesResistor * ((vRef / voltage) - 1.0);
        return avgResistance;
        }

        // Private: calculate temperature from resistance
        float calculateTemp(float resistance) {
        float steinhart = resistance / nominalRes;
        steinhart = log(steinhart);
        steinhart /= beta;
        steinhart += 1.0 / (nominalTemp + 273.15);
        steinhart = 1.0 / steinhart;
        steinhart -= 273.15;

        return steinhart + offsetTemp;
        }

    public:
        // Constructor with initializer list
        NTC(uint8_t adcPin, float adcMaxVal = 4095.0, float vRefVal = 3.6,
        float seriesR = 10000.0, float nominalR = 10000.0, float nominalT = 25.0,
        float betaVal = 3950.0, int samples = 20, float offset = -3.5f)
        : pin(adcPin), adcMax(adcMaxVal), vRef(vRefVal), seriesResistor(seriesR),
            nominalRes(nominalR), nominalTemp(nominalT), beta(betaVal),
            sampleCount(samples), offsetTemp(offset)
        {
        analogReadResolution(12);
        analogSetAttenuation(ADC_11db); // ESP32 0–3.3V full range
        }

        // Public: read temperature (°C)
        float readTemperature() {
        float voltage = readVoltage();
        float resistance = readResistance(voltage);
        return calculateTemp(resistance);
        }

        // Optional getters for voltage and resistance
        float getVoltage() { return avgVoltage; }
        float getResistance() { return avgResistance; }
    };
    //========================= End Class =============================//

    // NTC ntcSensor(35); // GPIO35 analog pin

    // void setup() {
    //     Serial.begin(115200);
    //     delay(1000);
    //     Serial.println("ESP32 NTC Temperature Sensor (OOP) Started");
    // }

    // void loop() {
    //     float tempC = ntcSensor.readTemperature();

    //     Serial.print("Temperature: "); Serial.print(tempC, 2); Serial.println(" °C");
    //     Serial.print("Voltage: "); Serial.print(ntcSensor.getVoltage(), 3); Serial.println(" V");
    //     Serial.print("Resistance: "); Serial.print(ntcSensor.getResistance(), 1); Serial.println(" Ohm");

    //     delay(2000);
    // }


#endif