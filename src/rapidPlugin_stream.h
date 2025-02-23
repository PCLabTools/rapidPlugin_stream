/**
 * @file rapidPlugin_stream.h
 * @author Larry Colvin (PCLabTools@github)
 * @brief 
 * @version 0.1
 * @date 2023-10-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef rapidPlugin_stream_h
#define rapidPlugin_stream_h

#ifndef rapidPlugin_stream_stack_size
#define rapidPlugin_stream_stack_size 128
#endif

#ifndef rapidPlugin_stream_interface_stack_size
#define rapidPlugin_stream_interface_stack_size 512
#endif

#include "rapidRTOS.h"

/**
 * @brief rapidPlugin responsible for handling a stream to protect it from
 * other plugins and processes from potentially cross-talking on the stream
 * by using a task to handle each stream request independently
 * 
 */
class rapidPlugin_stream : public Stream
{
  public:
    rapidPlugin_stream(const char* identity, Stream& stream = Serial);
    ~rapidPlugin_stream();
    BaseType_t run();
    BaseType_t runCore(UBaseType_t core);
    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;
    virtual size_t write(uint8_t byte) override;
    virtual size_t write(const uint8_t* bytes, size_t length) override;
    int print(const char* data);
    int println(const char* data);
    template<typename... Args> int printf(const char* data, Args... args);
    void stop();
    const char* cmd(const char* command, TickType_t timeout = portMAX_DELAY);
  
  private:
    static void main_loop(void*);
    static void interface_loop(void*);
    const char*_pID = "";                   // main task name
    char _iID[32];                          // interface task name buffer
    TaskHandle_t _taskHandle = NULL;        // main task handle reference
    TaskHandle_t _interfaceHandle = NULL;   // interface task handle reference
    QueueHandle_t _taskQueue = NULL;        // interface task incoming queue
    QueueHandle_t _taskResponse = NULL;     // interface task outgoing queue
    QueueHandle_t _txQueue = NULL;          // main task outgoing queue for stream
    QueueHandle_t _rxQueue = NULL;          // main task incoming queue for stream
    Stream& _streamRef = Serial;            // stream reference
};

/**
 * @brief Construct a new rapidPlugin stream::rapidPlugin stream object
 * 
 * @param identity string literal containing the task name
 * @param stream reference to the stream which the handler will control
 */
rapidPlugin_stream::rapidPlugin_stream(const char* identity, Stream& stream)
{
  _pID = identity;
  _streamRef = stream;
  _txQueue = xQueueCreate(256, sizeof(uint8_t));
  _rxQueue = xQueueCreate(256, sizeof(uint8_t));
}

/**
 * @brief Destroy the rapidPlugin stream::rapidPlugin stream object
 * 
 */
rapidPlugin_stream::~rapidPlugin_stream()
{
  //
}

/**
 * @brief Runs the main loop stream handling task.
 * This function registers the task with the manager and runs the interface loop
 * 
 * @return BaseType_t 1 = task run successful | 0 = task failed to start
 */
BaseType_t rapidPlugin_stream::run()
{
  uint32_t stackDepth = rapidPlugin_stream_stack_size;
  uint32_t interfaceDepth = rapidPlugin_stream_interface_stack_size;
  #ifdef BOARD_ESP32
  stackDepth = stackDepth * 4;
  interfaceDepth = interfaceDepth * 4;
  #endif
  sprintf(_iID, "i_%s", _pID);
  if (!rapidRTOS.getTaskHandle(_pID))
  {
    _taskQueue = xQueueCreate(1, sizeof(const char*));
    _taskResponse = xQueueCreate(1, sizeof(const char*));
    if(xTaskCreate(&main_loop, _pID, stackDepth, this, 1, &_taskHandle)\
    && xTaskCreate(&interface_loop, _iID, interfaceDepth, this, 1, &_interfaceHandle))\
    return (BaseType_t)rapidRTOS.reg(_taskHandle, _pID, &_taskQueue, &_taskResponse);
    else
    {
      vTaskDelete(_taskHandle);
      _taskHandle = NULL;
      vQueueDelete(_taskQueue);
      _taskQueue = NULL;
      vQueueDelete(_taskResponse);
      _taskResponse = NULL;
      return 0;
    }
  }
  return 0;
}

/**
 * @brief Runs the main loop stream handling task on the specified core.
 * This function registers the task with the manager and runs the interface loop
 * 
 * @param core core ID
 * @return BaseType_t 1 = task run successful | 0 = task failed to start
 */
BaseType_t rapidPlugin_stream::runCore(UBaseType_t core)
{
  uint32_t stackDepth = rapidPlugin_stream_stack_size;
  uint32_t interfaceDepth = rapidPlugin_stream_interface_stack_size;
  #ifdef BOARD_ESP32
  stackDepth = stackDepth * 4;
  interfaceDepth = interfaceDepth * 4;
  #endif
  sprintf(_iID, "i_%s", _pID);
  if (!rapidRTOS.getTaskHandle(_pID))
  {
    _taskQueue = xQueueCreate(1, sizeof(const char*));
    _taskResponse = xQueueCreate(1, sizeof(const char*));
    #ifdef BOARD_ESP32
    if(xTaskCreatePinnedToCore(&main_loop, _pID, stackDepth, this, 1, &_taskHandle, core)\
    && xTaskCreatePinnedToCore(&interface_loop, _iID, interfaceDepth, this, 1, &_interfaceHandle, core))\
    return (BaseType_t)rapidRTOS.reg(_taskHandle, _pID, &_taskQueue, &_taskResponse);
    #elif BOARD_TEENSY
    if(xTaskCreate(&main_loop, _pID, stackDepth, this, 1, &_taskHandle)\
    && xTaskCreate(&interface_loop, _iID, interfaceDepth, this, 1, &_interfaceHandle))\
    return (BaseType_t)rapidRTOS.reg(_taskHandle, _pID, &_taskQueue, &_taskResponse);
    #elif BOARD_STM32
    if(xTaskCreate(&main_loop, _pID, stackDepth, this, 1, &_taskHandle)\
    && xTaskCreate(&interface_loop, _iID, interfaceDepth, this, 1, &_interfaceHandle))\
    return (BaseType_t)rapidRTOS.reg(_taskHandle, _pID, &_taskQueue, &_taskResponse);
    #else
    if(xTaskCreateAffinitySet(&main_loop, _pID, stackDepth, this, 1, core, &_taskHandle)\
    && xTaskCreateAffinitySet(&interface_loop, _iID, interfaceDepth, this, 1, core, &_interfaceHandle))\
    return (BaseType_t)rapidRTOS.reg(_taskHandle, _pID, &_taskQueue, &_taskResponse);
    #endif
    else
    {
      vTaskDelete(_taskHandle);
      _taskHandle = NULL;
      vQueueDelete(_taskQueue);
      _taskQueue = NULL;
      vQueueDelete(_taskResponse);
      _taskResponse = NULL;
    return 0;
    }
  }
  return 0;
}

/**
 * @brief Main loop for handling the stream reading and writing
 * 
 * @param pModule pointer to calling object
 */
void rapidPlugin_stream::main_loop(void* pModule)
{
  rapidPlugin_stream* plugin = (rapidPlugin_stream*)pModule;
  char outputByte;
  char inputByte;
  for ( ;; )
  {
    while (xQueueReceive(plugin->_txQueue, &outputByte, 1))
    {
      plugin->_streamRef.write(outputByte);
    }
    while (plugin->_streamRef.available())
    {
      inputByte = plugin->_streamRef.read();
      xQueueSend(plugin->_rxQueue, &inputByte, 1);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Interface loop used to process incoming rapidFunction commands
 * 
 * @param pModule pointer to calling object
 */
void rapidPlugin_stream::interface_loop(void* pModule)
{
  rapidPlugin_stream* plugin = (rapidPlugin_stream*)pModule;
  char messageBuffer[rapidRTOS_DEFAULT_INTERFACE_BUFFER] = "";
  const char* incomingMessage = messageBuffer;
  const char* outgoingMessage = messageBuffer;
  for ( ;; )
  {
    xQueueReceive(plugin->_taskQueue, &incomingMessage, portMAX_DELAY);
    sprintf(messageBuffer, ""); // clear message buffer for outgoing message
    rapidFunction incoming = rapidRTOS.parse(incomingMessage);
    do
    {
      if (!strcmp(incoming.function, "identity"))
      {
        sprintf(messageBuffer, "%s", plugin->_pID);
        continue;
      }
      if (!strcmp(incoming.function, "stop"))
      {
        plugin->stop();
        break;
      }
      sprintf(messageBuffer, "unknown_function(%s)", incoming.function);
      rapidRTOS.printDebug(1, rapidDebug::ERROR, "%s: unknown_function(%s)\n", plugin->_pID, incoming.function);
    } while (false);
    outgoingMessage = messageBuffer;
    xQueueSend(plugin->_taskResponse, &outgoingMessage, portMAX_DELAY);
  }
}

/**
 * @brief Returns how many bytes are available on the stream to read
 * 
 * @return int number of bytes available
 */
int rapidPlugin_stream::available()
{
  return uxQueueMessagesWaiting(_rxQueue);
}

/**
 * @brief Reads a single byte from the stream
 * 
 * @return int read byte
 */
int rapidPlugin_stream::read()
{
  int readByte;
  xQueueReceive(_rxQueue, &readByte, 100 / portTICK_PERIOD_MS);
  return readByte;
}

/**
 * @brief Reads a single byte from the stream without removing it from the buffer
 * 
 * @return int read byte
 */
int rapidPlugin_stream::peek()
{
  int readByte;
  xQueuePeek(_rxQueue, &readByte, 100 / portTICK_PERIOD_MS);
  return readByte;
}

/**
 * @brief Flushes the stream
 * 
 */
void rapidPlugin_stream::flush()
{
  _streamRef.flush();
}

/**
 * @brief Writes a single byte to the stream
 * 
 * @param byte byte to write to stream
 * @return size_t number of bytes written
 */
size_t rapidPlugin_stream::write(uint8_t byte)
{
  xQueueSend(_txQueue, &byte, 100 / portTICK_PERIOD_MS);
  return 1;
}

/**
 * @brief Writes an array of bytes to the stream
 * 
 * @param bytes array of bytes to write to stream
 * @param length length of array of bytes
 * @return size_t number of bytes written
 */
size_t rapidPlugin_stream::write(const uint8_t* bytes, size_t length)
{
  for (size_t i = 0 ; i < length ; i++)
  {
    write(bytes[i]);
  }
  return length;
}

/**
 * @brief Prints a string to the stream
 * 
 * @param data string to print
 * @return int bytes written to stream
 */
int rapidPlugin_stream::print(const char* data)
{
  int bytesWritten = 0;
  while (*data)
  {
    write(*data);
    data++;
    bytesWritten++;
  }
  return bytesWritten;
}

/**
 * @brief Prints a string with an end of line character to the stream
 * 
 * @param data string to print
 * @return int bytes written to stream
 */
int rapidPlugin_stream::println(const char* data)
{
  char eolCharacter = '\n';
  int bytesWritten = print(data);
  xQueueSend(_txQueue, &eolCharacter, 100 / portTICK_PERIOD_MS);
  return bytesWritten + 1;
}

/**
 * @brief Prints a string to the stream which can be modified using arguments
 * 
 * @tparam Args any number of arguments
 * @param data string to print containing escape characters for override
 * @param args argument parameters that override the string escape characters
 * @return int number of bytes written to stream
 */
template<typename... Args> int rapidPlugin_stream::printf(const char* data, Args... args)
{
  char formattedData[256];
  sprintf(formattedData, data, args...);
  return print(formattedData);
}

/**
 * @brief Stops the rapidPlugin stream handler
 * 
 */
void rapidPlugin_stream::stop()
{
  if (_taskHandle)
  {
    if (_interfaceHandle) { vTaskDelete(_interfaceHandle); }
    vTaskDelete(_taskHandle);
    _taskHandle = NULL;
    rapidRTOS.dereg(_pID);
    vQueueDelete(_taskQueue);
    _taskQueue = NULL;
    vQueueDelete(_taskResponse);
    _taskResponse = NULL;
  }
}

/**
 * @brief Sends a rapidFunction command to the plugin
 * 
 * @param command string literal containing the rapidFunction command
 * @param timeout maximum wait time for response from command
 * @return const char* response from command
 */
const char* rapidPlugin_stream::cmd(const char* command, TickType_t timeout)
{
  xQueueSend(_taskQueue, &command, timeout);
  const char* response;
  xQueueReceive(_taskResponse, &response, timeout);
  return response;
}

#endif // rapidPlugin_stream_h