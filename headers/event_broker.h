#pragma once

// Creates and starts the event broker task (priority 4).
// Must be called after system_queues_init().
void event_broker_start(void);
