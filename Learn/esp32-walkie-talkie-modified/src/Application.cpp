#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>

#include "Application.h"
#include "I2SMEMSSampler.h"
#include "I2SOutput.h"
#include "EspNowTransport.h"
#include "OutputBuffer.h"
#include "config.h"

#include "GenericDevBoardIndicatorLed.h"

static void application_task(void *param)
{
  // delegate onto the application
  Application *application = reinterpret_cast<Application *>(param);
  application->loop();
}

Application::Application()
{
  // Set output buffer
  m_output_buffer = new OutputBuffer(300 * 16);

  // Use I2S mic input
  m_input = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_Config,128); 

  // Use I2S speaker output
  m_output = new I2SOutput(I2S_NUM_0, i2s_speaker_pins);  

  // Use ESP-NOW transport
  m_transport = new EspNowTransport(m_output_buffer,ESP_NOW_WIFI_CHANNEL); 
  m_transport->set_header(TRANSPORT_HEADER_SIZE,transport_header);

  // Set LED indicator to built in
  m_indicator_led = new GenericDevBoardIndicatorLed(); 

  if (I2S_SPEAKER_SD_PIN != -1)
  {
    pinMode(I2S_SPEAKER_SD_PIN, OUTPUT);
  }
}

void Application::begin()
{
  // show a flashing indicator that we are trying to connect
  m_indicator_led->set_default_color(0);
  m_indicator_led->set_is_flashing(true, 0xff0000);
  m_indicator_led->begin();
  
  // bring up WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Show MAC Address
  Serial.print("My MAC Address is: ");
  Serial.println(WiFi.macAddress());

  // do any setup of the transport
  m_transport->begin();

  // connected so show a solid green light
  m_indicator_led->set_default_color(0x00ff00);
  m_indicator_led->set_is_flashing(false, 0x00ff00);

  // setup the transmit button
  pinMode(GPIO_TRANSMIT_BUTTON, INPUT_PULLDOWN);

  // start off with i2S output running
  m_output->start(SAMPLE_RATE);

  // start the main task for the application
  TaskHandle_t task_handle;
  xTaskCreate(application_task, "application_task", 8192, this, 1, &task_handle);
}

// application task - coordinates everything
void Application::loop()
{
  int16_t *samples = reinterpret_cast<int16_t *>(malloc(sizeof(int16_t) * 128));
  // continue forever
  while (true)
  {
    // do we need to start transmitting?
    if (digitalRead(GPIO_TRANSMIT_BUTTON))
    {
      Serial.println("Started transmitting");
      m_indicator_led->set_is_flashing(true, 0xff0000);
      // stop the output as we're switching into transmit mode
      m_output->stop();
      // start the input to get samples from the microphone
      m_input->start();
      // transmit for at least 1 second or while the button is pushed
      unsigned long start_time = millis();
      while (millis() - start_time < 1000 || digitalRead(GPIO_TRANSMIT_BUTTON))
      {
        // read samples from the microphone
        int samples_read = m_input->read(samples, 128);
        // and send them over the transport
        for (int i = 0; i < samples_read; i++)
        {
          m_transport->add_sample(samples[i]);
        }
      }
      m_transport->flush();
      // finished transmitting stop the input and start the output
      Serial.println("Finished transmitting");
      m_indicator_led->set_is_flashing(false, 0xff0000);
      m_input->stop();
      m_output->start(SAMPLE_RATE);
    }
    // while the transmit button is not pushed and 1 second has not elapsed
    Serial.print("Started Receiving");
    digitalWrite(I2S_SPEAKER_SD_PIN, HIGH);
    unsigned long start_time = millis();
    while (millis() - start_time < 1000 || !digitalRead(GPIO_TRANSMIT_BUTTON))
    {
      // read from the output buffer (which should be getting filled by the transport)
      m_output_buffer->remove_samples(samples, 128);
      // and send the samples to the speaker
      m_output->write(samples, 128);
    }
    digitalWrite(I2S_SPEAKER_SD_PIN, LOW);
    Serial.println("Finished Receiving");
  }
}
