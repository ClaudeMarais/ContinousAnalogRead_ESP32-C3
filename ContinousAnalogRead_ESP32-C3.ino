// A simple Arduino project to show how to continously read analog values directly from the ADC (analog to digital converter) on
// an ESP32-C3. This is much much much faster than calling analogRead(), so it's useful if you need to read values at a high frequency.
//
// For this example, an IR transmitter LED and IR receiver is connected to the board and continously reads values from the IR receiver.
//
// ESP32 ADC documentation here:
// https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html

#include <driver/adc.h>   // Analog to digital converter APIs

// Pins for attaching the IR transmitter and receiver
int pinIRTransmitter = D6;
int pinIRReceiver = A0;

// Will be reading 256 values into a buffer all at once
static const int numSensorValues = 256;
static const int bufferSize = numSensorValues * 4;
uint8_t analogReadBuffer[bufferSize] = { 0 };

#define GET_UNIT(x) ((x>>3) & 0x1)

// Initialize continous / DMA mode on pin A0, which is GPIO2 and ADC1 Channel 2
bool InitializeContinousADC()
{
  uint16_t adc1_chan_mask = BIT(0) | BIT(0) | BIT(1);

  adc_digi_init_config_t adc_dma_config =
  {
      .max_store_buf_size = bufferSize,
      .conv_num_each_intr = numSensorValues,
      .adc1_chan_mask = adc1_chan_mask,
      .adc2_chan_mask = 0,
  };

  esp_err_t ret = adc_digi_initialize(&adc_dma_config);

  if (ret != ESP_OK)
  {
    return false;
  }

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = { 0 };

  adc_pattern[0].atten = ADC_ATTEN_DB_11;
  adc_pattern[0].channel = ADC1_CHANNEL_2;
  adc_pattern[0].unit = GET_UNIT(ADC1_CHANNEL_2);
  adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_digi_configuration_t dig_cfg =
  {
      .conv_limit_en = true,
      .conv_limit_num = min(255, numSensorValues),
      .pattern_num = 1,
      .adc_pattern = adc_pattern,
      .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };

  ret = adc_digi_controller_configure(&dig_cfg);
  
  if (ret != ESP_OK)
  {
    return false;
  }

  return true;
}

// Need to check if data is valid
bool IsDataValid(const adc_digi_output_data_t* pData)
{
    const unsigned int unit = pData->type2.unit;
    
    if (unit > 2)
    {
      return false;
    }

    if (pData->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
    {
      return false;
    }

    return true;
}

// Read lots of values into a buffer, instead of one by one using alanogRead()
int ReadAnalogValues()
{
  // Reset values in buffer
  memset(analogReadBuffer, 0xcc, bufferSize);

  // Start ADC
  adc_digi_start();

  // Read ADC values into buffer
  uint32_t numBytesRead = 0;   
  adc_digi_read_bytes(analogReadBuffer, bufferSize, &numBytesRead, ADC_MAX_DELAY);

  // Stop ADC
  adc_digi_stop();

  return numBytesRead;
}

// Get the min, max and average of the values
void DoSomethingWithTheValues(int numBytesRead)
{
  int numValues = 0;
  uint32_t minValue = 4095;    // 12-bit precision, so the maximum value is 4095
  uint32_t maxValue = 0;
  uint32_t total = 0;

  for (int i = 0; i < numBytesRead; i+=4)
  {
    adc_digi_output_data_t* pData = (adc_digi_output_data_t*)&analogReadBuffer[i];

    // Only process valid values
    if (IsDataValid(pData))
    {
      numValues++;

      uint32_t sensorValue = pData->type2.data;

      minValue = min(minValue, sensorValue);
      maxValue = max(maxValue, sensorValue);
      total += sensorValue;
    }
  }

  if (numValues > 0)
  {
    float average = float(total / numValues);
    Serial.printf("Num values = %d    Min value = %d    Max value = %d    Avg value = %f\n", numValues, minValue, maxValue, average);
  }
  else
  {
    Serial.println("No valid sensor values!");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n********* Continous Analog Read *********\n");

  pinMode(pinIRReceiver, INPUT);
  pinMode(pinIRTransmitter, OUTPUT);

  digitalWrite(pinIRTransmitter, HIGH);

  InitializeContinousADC();
}

void loop()
{
  int numBytesRead = ReadAnalogValues();
  DoSomethingWithTheValues(numBytesRead);
}

