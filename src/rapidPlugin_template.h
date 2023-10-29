/**
 * @file rapidPlugin_template.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef rapidPlugin_template_h
#define rapidPlugin_template_h

#include "rapidRTOS.h"

/**
 * @brief rapidPlugin top level description
 * 
 */
class rapidPlugin_template : public rapidPlugin
{
  public:
    rapidPlugin_template(const char* identity);
    BaseType_t run();
    BaseType_t runCore(BaseType_t core);

  private:
    static void main_loop(void*);
    uint8_t interface(rapidFunction incoming, char messageBuffer[]);
};

/**
 * @brief Construct a new rapidPlugin template::rapidPlugin template object
 * 
 * @param identity string literal containing task name
 */
rapidPlugin_template::rapidPlugin_template(const char* identity)
{
  _pID = identity;
}

/**
 * @brief Runs the main loop task.
 * rapidRTOS registers the task with the manager and creates the interface handlers
 * 
 * @return BaseType_t 1 = task run successful | 0 = task failed to start
 */
BaseType_t rapidPlugin_template::run()
{
  return rapidPlugin::run(&main_loop);
}

/**
 * @brief Runs the main loop task on the specified core.
 * rapidRTOS registers the task with the manager and creates the interface handlers
 * using the same core as the main loop
 * 
 * @param core core ID
 * @return BaseType_t 1 = task run successful | 0 = task failed to start
 */
BaseType_t rapidPlugin_template::runCore(BaseType_t core)
{
  return rapidPlugin::runCore(core, &main_loop);
}

/**
 * @brief main loop task
 * 
 * @param pModule pointer to the calling object
 */
void rapidPlugin_template::main_loop(void* pModule)
{
  rapidPlugin_template* plugin = (rapidPlugin_template*)pModule;
  for ( ;; )
  {
    // do something here
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Interface handler extended functions.
 * This function is to be used for creating custom states 
 * that are called when rapidFunction commands are received
 * 
 * @param incoming 
 * @param messageBuffer 
 * @return uint8_t 
 */
uint8_t rapidPlugin_template::interface(rapidFunction incoming, char messageBuffer[])
{
  do
  {
    if (!strcmp(incoming.function, "random"))
    {
      int min = 0, max = 0;
      sscanf(incoming.parameters, "%d,%d", &min, &max);
      int result = rand() % (max - min) + min;
      sprintf(messageBuffer, "random_example(%d,%d) = %d", min, max, result);
      continue;
    }
    rapidPlugin::interface(incoming, messageBuffer);
    return 0;
  } while (false);
  return 1;
}

#endif