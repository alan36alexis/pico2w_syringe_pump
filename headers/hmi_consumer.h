#pragma once

// Creates and starts the HMI consumer task (priority 2).
// Only creates the task if g_hmi_q != NULL (i.e. ENABLE_TFT was defined
// when system_queues_init() ran). Safe to call unconditionally.
// Must be called after system_queues_init().
void hmi_consumer_start(void);
