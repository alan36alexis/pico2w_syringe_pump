#pragma once

// Creates and starts the serial consumer task (priority 1).
// This is the only place in the firmware that maps SystemEventID_t to
// human-readable strings. Must be called after system_queues_init().
void serial_consumer_start(void);
